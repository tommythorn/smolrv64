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

# ubuntu-nfs.dtb IS the boot device tree: it is generated for the shipping clock
# (PROBE_CLK_DIV8=48) by ../ubuntu/Makefile and checked by src/lint.sh. (The hand-written
# ubuntu.dtb of the SD-card era, with its stale timebase-frequency, used to be the default;
# it is gone since the 2026-09 release.)
DTB=${DTB:-ubuntu-nfs.dtb}
FW=${FW:-fw_payload.bin}
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

# THE MODEL STRING NAMES THE BITSTREAM, NOT THE CHECKOUT (2026-09-07). ubuntu-nfs.dts is
# generated with git HEAD's short commit in `model`, which is the RTL on the board only when
# the boot follows a gate in the same tree and nothing was committed while it built (the
# W6 gate's tree moved two commits during its build). The monitor's banner carries the
# commit the bitstream was built from (rtl=...): when the last banner in the log names
# another one, the DTS is regenerated with it (--rtl=), so /proc/device-tree/model on the
# board answers "what is running" and a Geekbench result page names the right RTL.
if [[ "$DTB" == ubuntu-nfs.dtb && ( -n "${RTL_BANNER:-}" || -f "$LOG" ) ]]; then
    # board-gate.sh passes the banner it waited for (RTL_BANNER); alone, the last one in the log.
    banner_rtl=${RTL_BANNER:-$(grep -aoE 'rtl=[0-9a-f]{7,12}' "$LOG" | tail -1 | cut -d= -f2)}
    if [[ -n "$banner_rtl" ]] && ! grep -q "SmolRV64 ooo2 $banner_rtl @" ubuntu-nfs.dts 2>/dev/null; then
        echo "[ubuntu-boot] model string: the banner says rtl=$banner_rtl, regenerating $DTB with it"
        python3 ../../tools/check-dts-timebase.py --gen "$(sed -n 's/^DIV8 *?= *//p' Makefile)" \
            "--rtl=$banner_rtl" ubuntu-nfs.dts.in ubuntu-nfs.dts
        dtc -I dts -O dtb -o "$DTB" ubuntu-nfs.dts
    fi
fi
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
# SD loader is dead — XMODEM the firmware to FW_ADDR instead of the old SD load
# (`SL2800 ffff 80000000` / wait_for_loaded), which now fails "SD init failed".
send_file "$FW_ADDR"     "$FW"     $((base + 2))
# Optional initramfs. The old SD-loader path used to place this; when that died the
# upload was dropped but the banner above kept advertising it, so a DTB declaring
# linux,initrd-start/end would point the kernel at STALE DDR ("Freeing initrd memory"
# then panic, with nothing actually loaded). Send it when INITRD names a real file.
n=2
if [[ -n "${INITRD:-}" && -f "$INITRD" ]]; then
    n=3
    send_file "$INITRD_ADDR" "$INITRD" $((base + 3))
fi
send_line "X${FW_ADDR} 0 ${DTB_ADDR}"
echo "[ubuntu-boot] done"
