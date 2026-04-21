#!/usr/bin/env bash
# fpga-boot.sh — upload DTB + fw_payload into a running smolrv64 FPGA monitor
# via an existing `screen` session, then kick off execution.
#
# Prereqs:
#   * FPGA already programmed with monitor running at the serial console
#   * `screen -L` session already attached to the serial TTY (so screenlog.0 exists)
#   * `sx` (lrzsz) installed; `screen`'s exec escape (``!!``) enabled
#
# Usage:
#   ./fpga-boot.sh                     # autodetect first screen session
#   ./fpga-boot.sh <screen-session>    # e.g. 3673945.pts-3.coffee
#
# Exits non-zero if a transfer doesn't complete within the timeout.

set -euo pipefail

cd "$(dirname "$0")"

DTB=${DTB:-dts.dtb}
FW=${FW:-fw_payload.bin}
DTB_ADDR=${DTB_ADDR:-81000000}
FW_ADDR=${FW_ADDR:-80000000}
LOG=${LOG:-screenlog.0}
TIMEOUT=${TIMEOUT:-900}   # seconds per xmodem transfer

SESSION=${1:-$(screen -ls | awk '/\t[0-9]+\./ {print $1; exit}')}
if [[ -z "${SESSION:-}" ]]; then
    echo "ERROR: no screen session found; pass one as arg" >&2
    exit 1
fi

[[ -f "$DTB" ]] || { echo "ERROR: $DTB not found" >&2; exit 1; }
[[ -f "$FW"  ]] || { echo "ERROR: $FW not found"  >&2; exit 1; }
[[ -f "$LOG" ]] || { echo "ERROR: $LOG not found (is 'screen -L' active?)" >&2; exit 1; }

echo "[fpga-boot] session=$SESSION"
echo "[fpga-boot] sending $DTB -> 0x$DTB_ADDR, $FW -> 0x$FW_ADDR"

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

send_file() {
    local addr=$1 file=$2 target_count=$3
    echo "[fpga-boot]   Y$addr + sx -k $file"
    screen -S "$SESSION" -X stuff "Y${addr}\n"
    # give monitor a beat to print its "start XMODEM" prompt
    sleep 1
    screen -S "$SESSION" -X exec '!!' sx -k "$file"
    wait_for_new_complete "$target_count"
    echo "[fpga-boot]   done ($file)"
}

base=$(grep -c "Transfer complete" "$LOG" 2>/dev/null || true)
send_file "$DTB_ADDR" "$DTB" $((base + 1))
send_file "$FW_ADDR"  "$FW"  $((base + 2))

echo "[fpga-boot] launching: X${FW_ADDR} 0 ${DTB_ADDR}"
screen -S "$SESSION" -X stuff "X${FW_ADDR} 0 ${DTB_ADDR}\n"
echo "[fpga-boot] done."
