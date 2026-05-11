#!/usr/bin/env bash
set -euo pipefail

ROOT=$(git rev-parse --show-toplevel)
PLATFORM="$ROOT/platforms/rk-xcku5p-f-v1.2"
UBUNTU="$ROOT/workloads/ubuntu"
LOGDIR=${LOGDIR:-"/tmp/smolrv64-test-gate-$(date +%Y%m%d-%H%M%S)"}
BOOT_TIMEOUT=${BOOT_TIMEOUT:-1200}

mkdir -p "$LOGDIR"

log() {
    printf '[test-gate] %s\n' "$*"
}

run_logged() {
    local name=$1
    shift
    log "$name: $*"
    "$@" 2>&1 | tee "$LOGDIR/$name.log"
}

fail_dirty() {
    log "refusing to test uncommitted source state"
    git -C "$ROOT" status --short
    exit 1
}

HEAD_SHORT=$(git -C "$ROOT" rev-parse --short HEAD)
HEAD_FULL=$(git -C "$ROOT" rev-parse HEAD)

log "commit=$HEAD_SHORT ($HEAD_FULL)"
log "logdir=$LOGDIR"

git -C "$ROOT" diff --quiet --cached || fail_dirty

if ! git -C "$ROOT" diff --quiet -- \
    src \
    platforms/rk-xcku5p-f-v1.2/rk_xcku5p.srcs \
    platforms/rk-xcku5p-f-v1.2/build.tcl \
    platforms/rk-xcku5p-f-v1.2/Makefile \
    workloads/monitor \
    workloads/ubuntu/ubuntu.dts \
    workloads/ubuntu/ubuntu-boot.sh; then
    fail_dirty
fi

log "cleaning untracked platform artifacts"
git -C "$PLATFORM" clean -fd

run_logged bit make -C "$PLATFORM" bit

BIT="$PLATFORM/rk_xcku5p.runs/impl_1/rk_xcku5p.bit"
if [[ ! -f "$BIT" ]]; then
    log "missing bitstream: $BIT"
    exit 1
fi

TIMING="$PLATFORM/rk_xcku5p.runs/impl_1/rk_xcku5p_timing_summary_postroute_physopted.rpt"
if [[ ! -f "$TIMING" ]]; then
    log "missing timing report: $TIMING"
    exit 1
fi

if grep -q "timing constraints are not met" "$TIMING"; then
    log "timing failed according to $TIMING"
    exit 1
fi

grep -E "WNS|TNS|WHS|THS" "$TIMING" | head -20 | tee "$LOGDIR/timing-excerpt.log" || true

run_logged program make -C "$PLATFORM" program

log "boot: ./ubuntu-boot.sh"
(
    cd "$UBUNTU"
    timeout "$BOOT_TIMEOUT" ./ubuntu-boot.sh
) 2>&1 | tee "$LOGDIR/ubuntu-boot.log"

log "PASS commit=$HEAD_SHORT"
