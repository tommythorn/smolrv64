#!/usr/bin/env bash
# dma-stress-step.sh <bitstream> <tag> [secs] -- program, boot, run the directed DMA+CBO stress
# (/var/tmp/dmastress/dma-cbo-stress.sh on the board's NFS root), one verdict line:
#   DMA-STRESS: PASS <tag> rtl=<banner> <secs> s | FAIL <tag> rtl=<banner> -- <first fault line>
# The stress is the directed stand-in for "Geekbench 6 PDF Renderer at minute 90": NFS reads
# with the page cache dropped (non-coherent DMA + cbo.inval), anonymous page churn (cbo.zero),
# two rachk instances (deep stacks of saved ra, checksum self-check). Results in
# gate-results/dma-stress-<tag>/.
# NOBOOT=1: the board is already up with this bitstream; skip program/boot and just stress.
set -u
BIT=${1:?bitstream}; TAG=${2:?tag}; SECS=${3:-1200}
# The board's address changes per boot (DHCP). Its MAC does not: prefer the neighbour table
# (the NFS-peer lookup goes empty whenever the NFS TCP connection idles out, e.g. mid-Geekbench).
MAC=4a:18:30:e1:28:bc
board_ip() {
   local c
   for c in $(ip neigh | awk -v m=$MAC '$5==m && $6!="FAILED"{print $1}' | sort -u) \
            $(ss -tan state established '( sport = :2049 )' | awk 'NR>1{print $5}' | cut -d: -f1 | sort -u); do
      ssh -o ConnectTimeout=5 -o BatchMode=yes tommy@$c true 2>/dev/null && { echo $c; return 0; }
   done; return 1
}
cd "$(dirname "$0")/.."; REPO=$(pwd); RES=$REPO/gate-results/dma-stress-$TAG; mkdir -p "$RES"
say() { echo "[$(date +%H:%M:%S)] $*"; }
if [ -z "${NOBOOT:-}" ]; then
   say "programming $BIT as $TAG"
   BIT=$BIT STRESS_S=0 "$REPO/tools/board-gate.sh" "$RES" > "$RES/gate.log" 2>&1
   grep -q "BOARD: PASS" "$RES/gate.log" || { echo "DMA-STRESS: FAIL $TAG -- board did not boot: $(grep -m1 'BOARD: FAIL' "$RES/gate.log")"; exit 1; }
   RTL=$(grep -o 'rtl=[0-9a-f+]*' "$RES/gate.log" | tail -1)
   say "booted $RTL"
fi
ip=""; for i in $(seq 1 40); do ip=$(board_ip) && break; sleep 10; done
[ -n "$ip" ] || { echo "DMA-STRESS: FAIL $TAG -- no ssh to the board"; exit 1; }
RTL=${RTL:-$(ssh -o BatchMode=yes tommy@$ip 'cat /proc/device-tree/model' 2>/dev/null | tr -d '\0' | grep -o 'ooo2 [0-9a-f+]*' | sed 's/ooo2 /rtl=/')}
say "stress on $ip for $SECS s"
timeout $((SECS + 600)) ssh -o BatchMode=yes -o ServerAliveInterval=30 tommy@$ip "bash /var/tmp/dmastress/dma-cbo-stress.sh $SECS" > "$RES/stress.log" 2>&1
rc=$?
tail -n 4 "$RES/stress.log"
if grep -q "DMA-CBO-STRESS: PASS" "$RES/stress.log"; then echo "DMA-STRESS: PASS $TAG $RTL $SECS s"; exit 0; fi
echo "DMA-STRESS: FAIL $TAG $RTL (rc=$rc) -- $(grep -m1 -E 'unhandled signal|MISMATCH|DMA-CBO-STRESS: FAIL' "$RES/stress.log")"; exit 1
