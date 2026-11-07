#!/usr/bin/env bash
# The board half of tools/gate.sh, runnable alone: program a bitstream, NFS-boot Ubuntu, judge.
#
#   tools/board-gate.sh <resultdir>                    # the bitstream banked there, else impl_1's
#   BIT=/var/tmp/x.bit tools/board-gate.sh <resultdir> # another one
#   BOOT_WAIT=1500 ...                                 # seconds to allow for login: (default 1800)
#   REPLAY=<byte> tools/board-gate.sh <resultdir>       # judge an old boot from that console offset, no board
#   STRESS_S=900 ...                                  # seconds of post-login userspace stress (0 = skip)
#   STRESS_ONLY=1 tools/board-gate.sh <resultdir>        # no program/boot: run the stress on the board as it is
#   VGA=0 ...                                         # boot without the graphics console (default VGA=1)
#   LAT=0 ...                                         # skip the memory-latency chase after the stress
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
# is captured so the log proves WHICH RTL booted. A PASS also prints the board's memory latency
# (latency(), below): a record, never a verdict.
set -u
cd "$(dirname "$0")/.."
REPO=$(pwd); UB=$REPO/workloads/ubuntu
# The device tree and the payload that boot are THIS tree's (UB); the serial console (`screen -L`,
# screenlog.0) lives in ONE checkout's workloads/ubuntu -- the main one, which `git worktree list`
# prints first. A gate run from a worktree reads the console there (gate V4, 2026-09-05: "BOARD:
# FAIL (upload)" with the bitstream already on the board) and still boots its own tree.
# Three boards hang off coffee. BOARD=2g|4g|hdmi names the one under test and sets what must match
# it (tools/boards.sh): its platform, its console log and screen session, its JTAG cable, its device
# tree (the memory, and the NFS root: each board has its own, so two boards never write one tree)
# and the MAC its FPGA's DNA gives it, which the boot must show.
MAIN=$(git -C "$REPO" worktree list | head -1 | awk '{print $1}')
. "$REPO/tools/boards.sh" || { echo "BOARD=$BOARD: 2g, 4g or hdmi"; exit 2; }
PLAT=$REPO/$PLAT
export JTAG DTB DTB_ADDR=$DTBA
CON=${CON:-$UB/screenlog.0}
[ -f "$CON" ] || CON=$(git -C "$REPO" worktree list | head -1 | awk '{print $1}')/workloads/ubuntu/screenlog.0
# With several boards on one host, CON (the console log) and SCREEN (the `screen` session on
# that console's tty) name the board under test; JTAG=<cable serial> reaches `make program`.
# Unset, the first screen session, as with one board.
SCREEN=${SCREEN:-$(screen -ls | awk '/\t[0-9]+\./ {print $1; exit}')}
# Every command goes to the board through that session, and without it each `screen -X` only
# prints "No screen session found": the gate programmed the board and then watched a log that
# never grew (2026-11-04, after a power-on). The tty is named by the cable (ttyUSB numbers move).
if [ -z "${REPLAY:-}" ] && ! screen -ls | grep -qE "^[[:space:]]+[0-9]+\.${SCREEN:-NONE}[[:space:]]"; then
   echo "BOARD: FAIL (no screen session '${SCREEN:-}' on the console; start it:"
   echo "  screen -dmS ${SCREEN:-<name>} -L -Logfile $CON ${TTYDEV:-<its console tty>} $BAUD)"
   exit 1
fi
# The board's address is read off ITS console (the kernel's IP-Config line, from byte $1 on),
# never off the host's NFS peer list, which names every board.
con_ip() { tail -c +$(( ${1:-0} + 1 )) "$CON" | tr -d '\r' | grep -aoE 'ipaddr=192\.168\.1\.[0-9]+' | tail -1 | cut -d= -f2; }
RES=${1:?result dir}; BOOT_WAIT=${BOOT_WAIT:-1800}; mkdir -p "$RES"
# The bitstream: BIT=, else the one banked in the result directory (tools/gate.sh puts it there),
# else the platform's last build -- which is only this result's when the same tree built it.
[ -z "${BIT:-}" ] && [ -f "$RES/$PROJ.bit" ] && BIT=$(cd "$RES" && pwd)/$PROJ.bit
# make program runs in the platform directory: a given BIT= is read from the caller's.
[ -n "${BIT:-}" ] && BIT=$(realpath "$BIT")
# The board boots with the graphics console scanning out (ubuntu-boot.sh VGA): its memory
# traffic is part of the machine every bitstream is judged on.
export VGA=${VGA:-1}
BAD='Kernel panic|Oops \[#|Unable to handle kernel paging|unhandled signal|segfault|SIGSEGV|status=11/SEGV|core dumped|is not a head|NETDEV WATCHDOG|nfs: server .* not responding'
MARK='riscv: base ISA extensions'

# ---- the post-login userspace stress (B10) --------------------------------------------------
# The board is the NFS peer of this host on 192.168.1.x (see the memory note: its address is DHCP);
# sshd comes up a few minutes after login:. Geekbench runs its subtests in order under a timeout
# (124 = the budget ended, the expected exit); the verdict is the exit status plus every fault
# line dmesg and the console gained during the run.
# find_ip sets ip once the board's sshd answers there (within 400 s); ip stays visible to the
# integrity log and the latency chase, which run after the stress.
find_ip() {
   local i
   for i in $(seq 1 40); do
      ip=$(con_ip "${PRE:-0}")
      [ -n "$ip" ] && timeout 15 ssh -o BatchMode=yes -o ConnectTimeout=8 -o StrictHostKeyChecking=no tommy@$ip true 2>/dev/null && return 0
      sleep 10
   done
   ip=""; return 1
}
stress() {
   [ "${STRESS_S:-900}" -gt 0 ] || { echo "stress: skipped (STRESS_S=0)"; return 0; }
   local rc pre_d pre_c faults
   find_ip || { echo "BOARD: FAIL (stress: no ssh to the board within 400 s)"; return 1; }
   # The board must be running THIS tree's device tree: its ISA string is what Linux enables.
   local want got
   want=$(sed -n 's/.*riscv,isa = "\(.*\)";.*/\1/p' "$UB/ubuntu-nfs.dts.in" | head -1)
   got=$(timeout 20 ssh -o BatchMode=yes tommy@$ip 'tr -d "\0" < /proc/device-tree/cpus/cpu@0/riscv,isa' 2>/dev/null)
   [ "$got" = "$want" ] || { echo "BOARD: FAIL (the board's device tree says riscv,isa=$got, this tree's $want)"; return 1; }
   pre_d=$(timeout 20 ssh -o BatchMode=yes tommy@$ip 'dmesg | wc -l' 2>/dev/null); pre_c=$(wc -c < "$CON")
   echo "stress: $ip, ${STRESS_S:-900}s of Geekbench (dmesg lines before: $pre_d)"
   # Geekbench is its own session (setsid) so the budget's end kills the whole process group:
   # a plain `timeout` signals the parent and leaves its workers running on the board. The
   # session leader writes ITS OWN pid to a file and the kill reads it: \$! of a backgrounded
   # launch names a shell in the ssh command's group, not the leader of the new session
   # (observed: a `bash` of the command's group parenting a Geekbench in a session of its own),
   # and a group kill on that pid reaches nothing.
   timeout $(( ${STRESS_S:-900} + 120 )) ssh -o BatchMode=yes -o ServerAliveInterval=30 tommy@$ip \
      "cd ~/Geekbench-5.4.1-LinuxRISCVPreview && rm -f /var/tmp/gate-stress.pid && setsid sh -c 'echo \$\$ > /var/tmp/gate-stress.pid; exec ./geekbench_riscv64' > /var/tmp/gate-stress.log 2>&1 < /dev/null & sleep 3; P=\$(cat /var/tmp/gate-stress.pid); t=0; while kill -0 \$P 2>/dev/null && [ \$t -lt ${STRESS_S:-900} ]; do sleep 5; t=\$((t+5)); done; if kill -0 \$P 2>/dev/null; then kill -- -\$P; sleep 5; kill -9 -- -\$P 2>/dev/null; RC=124; else RC=0; fi; sleep 2; echo RC=\$RC left=\$(ps -o pid= -s \$P 2>/dev/null | wc -l); dmesg | tail -n +$((pre_d+1)) | grep -E 'Oops|BUG:|Unable to handle|unhandled signal|segfault|cause:|is not a head|NETDEV WATCHDOG'" > "$RES/stress.log" 2>&1
   rc=$(grep -o 'RC=[0-9]*' "$RES/stress.log" | tail -1 | cut -d= -f2)
   faults=$(( $(grep -cE 'Oops|BUG:|Unable to handle|unhandled signal|segfault|cause:|is not a head|NETDEV WATCHDOG' "$RES/stress.log") ))
   faults=$(( faults + $(tail -c +$((pre_c+1)) "$CON" | tr -d '\r' | grep -acE "$BAD") ))
   echo "stress: rc=${rc:-?} (124 = ran the whole budget) faults=$faults  subtests: $(timeout 20 ssh -o BatchMode=yes tommy@$ip 'grep -c "^  Running" /var/tmp/gate-stress.log' 2>/dev/null)"
   if [ "$faults" -eq 0 ] && { [ "${rc:-1}" = 124 ] || [ "${rc:-1}" = 0 ]; }; then errlog || return 1; return 0; fi
   grep -E 'Oops|BUG:|Unable to handle|unhandled signal|segfault|cause:|NETDEV' "$RES/stress.log" | head -4
   echo "BOARD: FAIL (stress: rc=${rc:-?} faults=$faults)"; return 1
}
# ---- the integrity log (rv_errlog; docs/SmolRV64-Spec.md) --------------------------------------
# The design's own invariants, latched in hardware and read back from Linux once the stress is
# over. A nonzero vector is a FAIL whatever the console says: a run can look clean and still
# have broken a rule the core relies on -- and this is the only gate that sees a violation
# whose consequence would have surfaced hours later. Only a read that says clean passes: a log
# that could not be read (a root without /var/tmp/errlog, no ssh) or a bitstream without one
# is a gate that saw nothing.
errlog() {
   local out; out=$("$REPO/tools/errlog-read.sh" "$ip" 2>&1); echo "$out"
   case "$out" in "errlog: clean"*) return 0;; esac
   echo "BOARD: FAIL (integrity log)"; return 1
}
# ---- the memory latency (tools/board/membench), a record and never a verdict --------------------
# No simulation sees the board's DRAM path (the DDR port's clock crossings, the MIG), so the gate
# measures it: chase2 counts its own user-mode cycles over 16 KiB (D$ hits), 256 KiB (64 pages,
# all in the dTLB's first level, most loads missing the D$) and 4 MiB (pages only the second level
# holds), and the D$ miss and the first-level dTLB miss are solved from its counted miss rates.
# It is built on the board in the same ssh session, and the 30 s after the build let the login's
# systemd --user settle: its CPU, and the TLBs every context switch empties, a chase counts.
latency() {
   [ "${LAT:-1}" != 0 ] || return 0
   [ -n "${ip:-}" ] || find_ip || { echo "latency: no ssh to the board"; return 0; }
   tar -C "$REPO/tools/board/membench" -cf - Makefile chase2.c pmu.h | timeout 300 ssh -o BatchMode=yes tommy@$ip \
      'D=/var/tmp/gate-latency; rm -rf $D && mkdir -p $D && tar -C $D -xf - && make -s -C $D chase2 >/dev/null && sleep 30 && for k in 16 256 4096; do for r in 1 2 3; do $D/chase2 $k 1 1000000; done; done' \
      > "$RES/latency.txt" 2>&1
   # Three runs a size, the fastest kept with its own miss rates: a background process that runs
   # during one of them evicts lines a 16 KiB chase never loses by itself.
   awk '/ KiB k=1 / { for (i = 2; i <= NF; i++) { if ($i == "cycles/step") x = $(i-1); if ($i == "DCMISS") y = $(i-1); if ($i == "DTLB_MISS") z = $(i-1) }
                     if (!($1 in c) || x < c[$1]) { c[$1] = x; m[$1] = y; t[$1] = z } }
        END { if (!(16 in c) || !(256 in c) || !(4096 in c) || m[256] < 0.1 || t[4096] < 0.1) { print "latency: unavailable (latency.txt)"; exit }
              h = c[16]; d = h + (c[256] - h) / m[256]; f = (c[4096] - h - m[4096] * (d - h)) / t[4096]
              printf "latency: D$ hit %.1f cycles, D$ miss %.1f (256 KiB, %.2f misses a load), a first-level dTLB miss +%.1f (4 MiB, %.2f a load); chase2, user-mode cycles\n", h, d, m[256], f, t[4096] }' "$RES/latency.txt"
}
if [ -n "${STRESS_ONLY:-}" ]; then RES=${1:?result dir}; mkdir -p "$RES"; stress && { echo "BOARD: PASS (stress only)"; exit 0; }; exit 1; fi

if [ -n "${REPLAY:-}" ]; then PRE=$REPLAY; BOOT_WAIT=0; echo "replay from byte $PRE"; else
PRE=$(wc -c < "$CON")
# Never program over a benchmark: a board that answers ssh and runs Geekbench is in use.
# FORCE_PROGRAM=1 overrides.
if [ -z "${FORCE_PROGRAM:-}" ]; then
   for c in $(con_ip 0); do
      if timeout 15 ssh -o BatchMode=yes -o ConnectTimeout=8 -o StrictHostKeyChecking=no tommy@$c 'pgrep -f "[g]eekbench"' >/dev/null 2>&1; then
         echo "BOARD: REFUSED (Geekbench is running on $c; FORCE_PROGRAM=1 to program anyway)"; exit 2
      fi
   done
fi
# Halt the board's Linux before programming: reconfiguring the FPGA under a running system cuts
# the SD card off mid-transfer, and a card left that way answers commands but sends no data until
# it is power-cycled. This board only (the last address its console printed), and only when what
# answers there is a board: a Simmerv guest on the same NFS root reports "SmooolRV64".
for c in $(con_ip 0); do
   m=$(timeout 15 ssh -o BatchMode=yes -o ConnectTimeout=8 -o StrictHostKeyChecking=no tommy@$c 'tr -d "\0" < /proc/device-tree/model' 2>/dev/null)
   case "$m" in "SmolRV64 "*)
      echo "halting $c ($m)"
      # A halt that goes ahead drops the connection (ssh's 255) or outlasts the timeout (124); any
      # other status is halt or sudo refusing, and its words are the reason the board stays up.
      hm=$(timeout 15 ssh -o BatchMode=yes tommy@$c 'sudo halt' 2>&1); hrc=$?
      case $hrc in 0|124|255) ;; *) echo "BOARD: FAIL ('sudo halt' on $c exited $hrc: $hm)"; exit 1;; esac
      # A shutdown can take minutes (systemd waits out a stuck udev worker, 90 s at a time), and
      # programming before the halt line is the cut above: wait it out, or give up unprogrammed.
      for i in $(seq 600); do tail -c +$((PRE+1)) "$CON" | tr -d '\0' | grep -aq "reboot: System halted" && break; sleep 1; done
      tail -c +$((PRE+1)) "$CON" | tr -d '\0' | grep -aq "reboot: System halted" \
         || { echo "BOARD: FAIL (no 'reboot: System halted' on the console 600 s after halting $c, which said '${hm}' (exit $hrc): not programming a running board)"; exit 1; }
      echo "halted after ${i} s"
      PRE=$(wc -c < "$CON");;
   esac
done
( cd "$PLAT" && timeout 900 make program ${BIT:+BIT=$BIT} ) > "$RES/program.log" 2>&1
grep -qi "programmed successfully" "$RES/program.log" || { echo "BOARD: FAIL (program)"; tail -5 "$RES/program.log"; exit 1; }
echo "programmed ${BIT:-impl_1}"
# The monitor's banner names the bitstream just programmed; wait for it past the offset (the
# console log flushes late) and hand it to ubuntu-boot.sh, whose model-string stamp otherwise
# reads the PREVIOUS build's banner (W7's board turn, 2026-09-07: the DTB said 9b3050f9 on
# a 6c1eeff1 bitstream).
for i in $(seq 1 30); do
   RTL_BANNER=$(tail -c +$((PRE+1)) "$CON" | tr -d '\r' | grep -aoE 'rtl=[0-9a-f]{7,12}\+?' | tail -1 | cut -d= -f2)
   [ -n "$RTL_BANNER" ] && break; sleep 2
done
echo "banner: rtl=${RTL_BANNER:-?}"
# The monitor waits at its prompt; a carriage return makes sure the line it is reading is empty
# before the upload below.
screen -S "$SCREEN" -X stuff $'\r'
# A TRAILING '+' MEANS THE BITSTREAM IS NOT THAT COMMIT. The monitor appends it from the
# build-id's source-dirty word; this gate used to grep only [0-9a-f] and silently drop it,
# so a dirty build was reported -- and remembered -- as the commit it was built on top of.
# 2026-09-19: a GB5 crash was attributed to 493dc343 for exactly this reason; the bitstream
# was 493dc343 + uncommitted C4a step 2. Never record a dirty build as a commit's verdict.
case "${RTL_BANNER:-}" in
   *+) echo "WARNING: the loaded bitstream is a DIRTY tree (${RTL_BANNER}) -- its verdict belongs to no commit" ;;
esac
export RTL_BANNER
# every device tree's model string names the LOADED bitstream (its banner), never the tree's
# commit: `make dtbs RTL=` for the variants, ubuntu-boot.sh again for the one it boots
( cd "$UB" && touch ubuntu-nfs.dts.in && make dtbs RTL="$RTL_BANNER" ) > "$RES/dtbs.log" 2>&1 \
   || { echo "BOARD: FAIL (dtbs)"; tail -3 "$RES/dtbs.log"; exit 1; }
XMODEM=""
if [ -n "${BOARD:-}" ]; then
   # A named board boots over TFTP, as the monitor's `go` does: this tree's payload and the board's
   # device tree go to coffee's /srv/tftp/$BOARD with the boot.txt `go $BOARD` runs: 19.7 MB in a
   # few seconds; XMODEM at the console's baud takes a minute or two. VGA=1 turns the scanout on, as ubuntu-boot.sh does.
   T=/srv/tftp/$BOARD
   cp "$UB/fw_payload.bin" "$T/fw_payload.bin" && cp "$UB/$DTB" "$T/smolrv64.dtb" \
      || { echo "BOARD: FAIL (staging $T)"; exit 1; }
   { echo "# The gate's boot of $RTL_BANNER from $REPO ($DTB), staged by tools/board-gate.sh."
     echo "N80000000 fw_payload.bin"; echo "N$DTBA smolrv64.dtb"
     # the monitor turns the scanout on at every reset, in the mode its V command last set, and tells
     # the device tree that mode at the jump (workloads/monitor/video.c); VGA=0 turns it off
     [ "$VGA" = 0 ] && echo "WW10005000 0"
     echo "X80000000 0 $DTBA"; } > "$T/boot.txt"
   cp "$T/boot.txt" "$RES/boot.txt"
   # a character at a time: a bitstream older than the UART's receive FIFO drops a burst
   GO=$(wc -c < "$CON")
   go="go $BOARD"; for ((i = 0; i < ${#go}; i++)); do screen -S "$SCREEN" -X stuff "${go:$i:1}"; sleep 0.02; done
   screen -S "$SCREEN" -X stuff $'\r'
   # The router's DHCP pool runs dry now and then and the monitor says so ("no DHCP lease"); the
   # board then boots the same payload and device tree over XMODEM, as an unnamed board does.
   for i in $(seq 60); do
      NET=$(tail -c +$((GO+1)) "$CON" | tr -d '\0' | grep -aoE "from DHCP|no DHCP lease" | head -1)
      [ -n "$NET" ] && break; sleep 1
   done
   [ "$NET" = "no DHCP lease" ] && { echo "go $BOARD: no DHCP lease, booting over XMODEM"; XMODEM=1; }
else
   XMODEM=1
fi
if [ -n "${XMODEM:-}" ]; then
   # the payload and device tree over XMODEM (ubuntu-boot.sh)
   ( cd "$UB" && LOG="$CON" timeout 3000 ./ubuntu-boot.sh "$SCREEN" ) > "$RES/upload.log" 2>&1 \
      || { echo "BOARD: FAIL (upload)"; tail -3 "$RES/upload.log"; exit 1; }
fi
fi
echo "booting, watching past byte $PRE"
# The boot's own marker: its byte offset anchors everything below.
END=$(( $(date +%s) + BOOT_WAIT )); POS=""
while :; do
   OFF=$(tail -c +$((PRE+1)) "$CON" | grep -abm1 "$MARK" | cut -d: -f1)
   [ -n "$OFF" ] && { POS=$((PRE+OFF)); break; }
   [ "$(date +%s)" -ge "$END" ] && break; sleep 10
done
[ -n "$POS" ] || { echo "BOARD: FAIL (no kernel marker past byte $PRE within ${BOOT_WAIT}s)"; exit 1; }
echo "kernel marker at byte $POS"
while [ "$(date +%s)" -lt "$END" ]; do
   NEW=$(tail -c +$((POS+1)) "$CON" | tr -d '\r')
   printf '%s' "$NEW" | grep -aq "login:" && break
   printf '%s' "$NEW" | grep -aqE "$BAD" && break
   sleep 10
done
NEW=$(tail -c +$((POS+1)) "$CON" | tr -d '\r')
# A replayed boot ends where the next programming's monitor banner begins.
[ -n "${REPLAY:-}" ] && NEW=$(printf '%s' "$NEW" | awk '/smolrv64 monitor/{exit} {print}')
printf '%s' "$NEW" > "$RES/boot.log"
RTL=$(tail -c +$((PRE+1)) "$CON" | tr -d '\r' | grep -aoE 'rtl=[0-9a-f]+\+?' | head -1)   # the banner precedes the marker
FAULTS=$(printf '%s' "$NEW" | grep -acE "$BAD")
LOGIN=$(printf '%s' "$NEW" | grep -ac "login:")
echo "login: $LOGIN   faults: $FAULTS   ${RTL:-rtl=?}   last: $(printf '%s' "$NEW" | grep -aoE '^\[ *[0-9]+\.[0-9]+\]' | tail -1)"
# The board that booted is the one named: the kernel's IP-Config line carries its MAC.
HW=$(printf '%s' "$NEW" | grep -aoE 'hwaddr=[0-9a-f:]+' | head -1 | cut -d= -f2)
if [ -n "$BMAC" ] && [ "$HW" != "$BMAC" ]; then
   echo "BOARD: FAIL (BOARD=$BOARD booted with MAC ${HW:-?}, not its $BMAC: another board, or another board's device tree)"; exit 1
fi
if [ "$LOGIN" -ge 1 ] && [ "$FAULTS" -eq 0 ]; then
   if [ -n "${REPLAY:-}" ]; then echo "BOARD: PASS"; exit 0; fi
   stress || exit 1
   latency
   echo "BOARD: PASS"; exit 0
fi
printf '%s' "$NEW" | grep -aE "$BAD|epc :" | head -4
echo "BOARD: FAIL login=$LOGIN faults=$FAULTS"; exit 1
