#!/usr/bin/env bash
# gb5-boot.sh — boot the Geekbench 5 initramfs on the smolrv64 FPGA.
#
# Same monitor/XMODEM flow as ubuntu/ubuntu-boot.sh, but three pieces instead of two:
# the 164 MB initramfs is uploaded too, so the board needs no network, no NFS and
# no Ubuntu.  Busybox runs /etc/init.d/S90geekbench, which execs the benchmark, so
# AES-XTS (workload 101, the one that fails validation) starts seconds after boot.
#
# The payoff is that this is the SAME image `make ref` boots under simmerv, so the
# hardware run and the golden reference are executing identical code on identical
# data — which is what "matching exactly what Geekbench 5 is doing" requires.
#
# At 3 Mbaud the initramfs takes roughly 10-15 minutes to upload.
#
# Prereqs: FPGA programmed (make -C platforms/rk-xcku5p-f-v1.2 program), and a
# `screen -L` session attached to the serial TTY.

set -euo pipefail
cd "$(dirname "$0")"

DTB=${DTB:-gb5-fpga.dtb}
FW=${FW:-fw_payload.bin}
INITRD=${INITRD:-img.Geekbench5.cpio}
DTB_ADDR=${DTB_ADDR:-82000000}
FW_ADDR=${FW_ADDR:-80000000}
INITRD_ADDR=${INITRD_ADDR:-96270e00}    # must match linux,initrd-start in the DTB
# The screen session was started in workloads/ubuntu, and `screen -X exec` runs sx
# in THAT cwd -- so its log lives there and every file handed to sx must be absolute.
LOG=${LOG:-../ubuntu/screenlog.0}
TIMEOUT=${TIMEOUT:-3600}                # the initramfs alone is ~164 MB
CHAR_DELAY=${CHAR_DELAY:-0.02}

SESSION=${1:-$(screen -ls | awk '/\t[0-9]+\./ {print $1; exit}')}
[[ -n "${SESSION:-}" ]] || { echo "ERROR: no screen session found" >&2; exit 1; }

for f in "$DTB" "$FW" "$INITRD"; do
    [[ -f "$f" ]] || { echo "ERROR: $f not found" >&2; exit 1; }
done
[[ -f "$LOG" ]] || { echo "ERROR: $LOG not found (is 'screen -L' active?)" >&2; exit 1; }

echo "[gb5-boot] session=$SESSION"
echo "[gb5-boot] $DTB -> 0x$DTB_ADDR, $INITRD -> 0x$INITRD_ADDR, $FW -> 0x$FW_ADDR"

wait_for_new_complete() {
    local target=$1 deadline=$(( SECONDS + TIMEOUT )) n
    while (( SECONDS < deadline )); do
        n=$(grep -c "Transfer complete" "$LOG" 2>/dev/null || true)
        (( n >= target )) && return 0
        sleep 5
    done
    echo "ERROR: transfer did not complete within ${TIMEOUT}s (target=$target)" >&2
    return 1
}

send_line() {
    local line=$1 i ch
    for ((i = 0; i < ${#line}; i++)); do
        ch=${line:i:1}
        screen -S "$SESSION" -X stuff "$ch"
        sleep "$CHAR_DELAY"
    done
    screen -S "$SESSION" -X stuff $'\n'
}

send_file() {
    local addr=$1 file=$2 target_count=$3
    local abs; abs=$(realpath "$file")
    echo "[gb5-boot]   Y$addr + sx -k $abs   ($(stat -c%s "$file") bytes)"
    send_line "Y${addr}"
    sleep 1
    screen -S "$SESSION" -X exec '!!' sx -k "$abs"
    wait_for_new_complete "$target_count"
    echo "[gb5-boot]   done ($file) at $(date +%T)"
}

base=$(grep -c "Transfer complete" "$LOG" 2>/dev/null || true)
send_file "$DTB_ADDR"    "$DTB"    $((base + 1))
send_file "$INITRD_ADDR" "$INITRD" $((base + 2))
send_file "$FW_ADDR"     "$FW"     $((base + 3))

echo "[gb5-boot] launching: X${FW_ADDR} 0 ${DTB_ADDR}"
send_line "X${FW_ADDR} 0 ${DTB_ADDR}"
echo "[gb5-boot] done at $(date +%T)."
