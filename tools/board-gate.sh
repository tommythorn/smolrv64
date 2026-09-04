#!/usr/bin/env bash
# The board half of tools/gate.sh, runnable alone: program a bitstream, NFS-boot Ubuntu, judge.
#
#   tools/board-gate.sh <resultdir>                    # impl_1's bitstream
#   BIT=/var/tmp/x.bit tools/board-gate.sh <resultdir> # a banked one
#   BOOT_WAIT=1500 ...                                 # seconds to allow for login: (default 1800)
#
# PASS = `login:` reached with ZERO faults. FAIL fast on the first line that already decides
# it: a kernel panic or Oops, a userspace fault, or virtio dying ("id N is not a head!",
# NETDEV WATCHDOG) -- the last two sat unrecognised for ten minutes per boot on 2026-09-04
# while the gate waited for a login that could not come. The console is cumulative across
# boots, so only bytes past the offset recorded before programming are read; and the
# monitor's `rtl=` line is captured so the log proves WHICH RTL booted.
set -u
cd "$(dirname "$0")/.."
REPO=$(pwd); PLAT=$REPO/platforms/rk-xcku5p-f-v1.2; UB=$REPO/workloads/ubuntu
RES=${1:?result dir}; BOOT_WAIT=${BOOT_WAIT:-1800}; mkdir -p "$RES"
BAD='Kernel panic|Oops \[#|Unable to handle kernel paging|unhandled signal|segfault|SIGSEGV|status=11/SEGV|core dumped|is not a head|NETDEV WATCHDOG'

PRE=$(wc -c < "$UB/screenlog.0")
( cd "$PLAT" && timeout 900 make program ${BIT:+BIT=$BIT} ) > "$RES/program.log" 2>&1
grep -qi "programmed successfully" "$RES/program.log" || { echo "BOARD: FAIL (program)"; tail -5 "$RES/program.log"; exit 1; }
echo "programmed ${BIT:-impl_1}"
( cd "$UB" && touch ubuntu-nfs.dts.in && make dtbs >/dev/null 2>&1; timeout 3000 ./ubuntu-boot.sh ) > "$RES/upload.log" 2>&1 \
   || { echo "BOARD: FAIL (upload)"; tail -3 "$RES/upload.log"; exit 1; }
echo "booting, watching past byte $PRE"
END=$(( $(date +%s) + BOOT_WAIT ))
while [ "$(date +%s)" -lt "$END" ]; do
   NEW=$(tail -c +$((PRE+1)) "$UB/screenlog.0" | tr -d '\r')
   printf '%s' "$NEW" | grep -aq "login:" && break
   printf '%s' "$NEW" | grep -aqE "$BAD" && break
   sleep 10
done
NEW=$(tail -c +$((PRE+1)) "$UB/screenlog.0" | tr -d '\r')
printf '%s' "$NEW" > "$RES/boot.log"
RTL=$(printf '%s' "$NEW" | grep -aoE 'rtl=[0-9a-f]+' | head -1)
FAULTS=$(printf '%s' "$NEW" | grep -acE "$BAD")
LOGIN=$(printf '%s' "$NEW" | grep -ac "login:")
echo "login: $LOGIN   faults: $FAULTS   ${RTL:-rtl=?}   last: $(printf '%s' "$NEW" | grep -aoE '^\[ *[0-9]+\.[0-9]+\]' | tail -1)"
if [ "$LOGIN" -ge 1 ] && [ "$FAULTS" -eq 0 ]; then echo "BOARD: PASS"; exit 0; fi
printf '%s' "$NEW" | grep -aE "$BAD|epc :" | head -4
echo "BOARD: FAIL login=$LOGIN faults=$FAULTS"; exit 1
