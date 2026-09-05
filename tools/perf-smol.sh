#!/usr/bin/env bash
# Run a command under perf with a smolrv64 event set that FITS IN THE COUNTERS.
#
# The DTB exposes 13 programmable counters (riscv,raw-event-to-mhpmcounters ... 0x0000fff8
# = mhpmcounter3..15).  cycles and instructions ride the dedicated mcycle/minstret and are
# free.  Ask for more than 13 raw events and perf MULTIPLEXES: every count becomes a scaled
# estimate and the CPI-stack identity stops closing.  Hence sets, not one giant list.
#
#   ./perf-smol.sh cpi  ./prog      CPI stack: every stall cause, the frontend breakdown,
#                                   ROB-full, the redirect drain, redirects, D$ misses (13)
#   ./perf-smol.sh mem  ./prog      D$/I$ traffic, loads/stores, dTLB walks            ( 8)
#   ./perf-smol.sh br   ./prog      redirect causes + the drain                        ( 5)
#   ./perf-smol.sh fb   ./prog      fetch-buffer payoff + frontend bubbles              ( 7)
#   ./perf-smol.sh all  ./prog      everything -- MULTIPLEXED, estimates only          (26)
#
# Running only the FE_* events (r0310-r0314) is NOT a CPI stack: every backend stall then
# lands in "retire" and the tool reports it as unattributed.  Use 'cpi'.
#
# Pipe the output through tools/perf-cpi-stack.py.
#
# PERF_TIMEOUT_MS=10800000 ./perf-smol.sh cpi ./long-workload   # 3 h slice, counters printed
set -u
SET=${1:-cpi}; shift || true

case "$SET" in
  # ST_MEM..ST_ROB + FE_MMU/FE_IC/FE_ALN/FE_QUE + RD_WAIT + REDIR + DCMISS = 13, the whole
  # counter file.  FE_BUB (r0310) is deliberately NOT here: it is the sum of the four FE_*
  # causes, so the parser reconstructs it and we spend the counter on something we cannot
  # derive.  r0305 (ROB full) and r0317 (the mispredict drain) are stack terms since
  # 2026-09-05; the set that lacked them read "issue / retire" 10 points too high.
  cpi) EV=r0300,r0301,r0302,r0303,r0304,r0305,r0311,r0312,r0313,r0314,r0317,r0005,r0102 ;;
  # + the dTLB: walks begun (r0104) and cycles walking (r0318), a subset of ST_MEM.
  mem) EV=r0003,r0004,r0100,r0102,r0110,r0112,r0104,r0318 ;;
  br)  EV=r0005,r0006,r0007,r0008,r0317 ;;
  # Does the fetch-buffer address comparison pay?  FB_RHIT counts hits in the redirect
  # shadow -- exactly what flush-on-redirect would turn into misses.  Paired with FE_* so the
  # cost side (queue-empty, no-bytes) is measured in the same pass.
  fb)  EV=r0005,r0315,r0316,r0311,r0312,r0313,r0314 ;;
  all) EV=r0003,r0004,r0005,r0006,r0007,r0008,r0100,r0102,r0104,r0110,r0112,r0300,r0301,r0302,r0303,r0304,r0305,r0310,r0311,r0312,r0313,r0314,r0315,r0316,r0317,r0318
       echo "warning: 26 raw events > 13 counters -- perf will multiplex and every count is" >&2
       echo "         a scaled ESTIMATE.  The CPI-stack residual will not close.  Prefer 'cpi'." >&2 ;;
  *)   echo "usage: $0 {cpi|mem|br|fb|all} COMMAND..." >&2; exit 2 ;;
esac

# No sudo if the admin lowered perf_event_paranoid (see tools/perf-smol-setup.sh); fall back
# to sudo only when the kernel actually refuses.
SUDO=""
[ "$(cat /proc/sys/kernel/perf_event_paranoid 2>/dev/null || echo 2)" -gt 1 ] && SUDO="sudo"

# PERF_TIMEOUT_MS: stop after N ms and PRINT.  Use perf's own --timeout, never an external
# `timeout`: SIGTERM kills perf before it emits the counter block, and SIGINT to perf is
# ignored while it waits on the workload -- a three-hour Geekbench slice was lost to exactly
# that, and the log looks merely truncated rather than failed.  The only other reliable
# lever is killing the WORKLOAD, which makes perf print normally.
exec $SUDO perf stat ${PERF_TIMEOUT_MS:+--timeout "$PERF_TIMEOUT_MS"} -e "cycles,instructions,$EV" "$@"
