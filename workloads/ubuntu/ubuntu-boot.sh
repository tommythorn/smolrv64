#!/usr/bin/env bash
# ubuntu-boot.sh — upload DTB + fw_payload into a running smolrv64 FPGA monitor
# via an existing `screen` session, then kick off execution.
#
# Prereqs:
#   * FPGA already programmed with monitor running at the serial console
#   * `screen -L` session already attached to the serial TTY (so screenlog.0 exists)
#   * `sx` (lrzsz) installed; `screen`'s exec escape (``!!``) enabled
#
# Usage:
#   ./ubuntu-boot.sh                     # autodetect first screen session
#   ./ubuntu-boot.sh <screen-session>    # e.g. 3673945.pts-3.coffee
#
# Exits non-zero if a transfer doesn't complete within the timeout.

set -euo pipefail

cd "$(dirname "$0")"

DTB=${DTB:-ubuntu.dtb}
FW=${FW:-fw_payload.bin-1.7+v7.1-rc1}
INITRD=${INITRD:-tiny128.cpio}
DTB_ADDR=${DTB_ADDR:-fffff000}
FW_ADDR=${FW_ADDR:-80000000}
INITRD_ADDR=${INITRD_ADDR:-ff62b000}
LOG=${LOG:-screenlog.0}
TIMEOUT=${TIMEOUT:-900}   # seconds per xmodem transfer
CHAR_DELAY=${CHAR_DELAY:-0.02}

if [[ "$DTB" == *.dtb ]]; then
    DTS="${DTB%.dtb}.dts"
    if [[ -f "$DTS" && ( ! -f "$DTB" || "$DTS" -nt "$DTB" ) ]]; then
        command -v dtc >/dev/null || { echo "ERROR: $DTS is newer than $DTB but dtc is not installed" >&2; exit 1; }
        echo "[ubuntu-boot] regenerating $DTB from $DTS"
        dtc -I dts -O dtb -o "$DTB" "$DTS"
    fi
fi

SESSION=${1:-$(screen -ls | awk '/\t[0-9]+\./ {print $1; exit}')}
if [[ -z "${SESSION:-}" ]]; then
    echo "ERROR: no screen session found; pass one as arg" >&2
    exit 1
fi

[[ -f "$DTB" ]] || { echo "ERROR: $DTB not found" >&2; exit 1; }
[[ -f "$FW"  ]] || { echo "ERROR: $FW not found"  >&2; exit 1; }
[[ -f "$LOG" ]] || { echo "ERROR: $LOG not found (is 'screen -L' active?)" >&2; exit 1; }

echo "[ubuntu-boot] session=$SESSION"
echo "[ubuntu-boot] sending $DTB -> 0x$DTB_ADDR, $FW -> 0x$FW_ADDR, $INITRD -> 0x$INITRD_ADDR"

wait_for_new_complete() {
    # Wait until grep -c "Transfer complete" $LOG reaches $1.
    local target=$1
    local deadline=$(( SECONDS + TIMEOUT ))
    while (( SECONDS < deadline )); do
        local n
        n=$(grep -c "Transfer complete" "$LOG" 2>/dev/null || true)
        if (( n >= target )); then return 0; fi
        sleep 2
    done
    echo "ERROR: transfer did not complete within ${TIMEOUT}s (target=$target)" >&2
    return 1
}

wait_for_loaded() {
    # Wait until grep -c "loaded sectors=" $LOG reaches $1.
    local target=$1
    local deadline=$(( SECONDS + TIMEOUT ))
    while (( SECONDS < deadline )); do
        local n
        n=$(grep -c "loaded sectors=" "$LOG" 2>/dev/null || true)
        if (( n >= target )); then return 0; fi
        sleep 2
    done
    echo "ERROR: transfer did not complete within ${TIMEOUT}s (target=$target)" >&2
    return 1
}

send_file() {
    local addr=$1 file=$2 target_count=$3
    echo "[ubuntu-boot]   Y$addr + sx -k $file"
    send_line "Y${addr}"
    # give monitor a beat to print its "start XMODEM" prompt
    sleep 1
    screen -S "$SESSION" -X exec '!!' sx -k "$file"
    wait_for_new_complete "$target_count"
    echo "[ubuntu-boot]   done ($file)"
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

base=$(grep -c "Transfer complete" "$LOG" 2>/dev/null || true)
send_file "$DTB_ADDR"    "$DTB"    $((base + 1))
base=$(grep -c "loaded sectors=" "$LOG" 2>/dev/null || true)
send_line "SL2800 e0b0 80000000"
wait_for_loaded $((base + 1))
send_line "X${FW_ADDR} 0 ${DTB_ADDR}"
echo "[ubuntu-boot] done"
