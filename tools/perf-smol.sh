#!/usr/bin/env bash
# Run a command under perf with a smolrv64 event set that FITS IN THE COUNTERS.
#
# The DTB exposes 13 programmable counters (riscv,raw-event-to-mhpmcounters ... 0x0000fff8
# = mhpmcounter3..15).  cycles and instructions ride the dedicated mcycle/minstret and are
# free.  Ask for more than 13 raw events and perf MULTIPLEXES: every count becomes a scaled
# estimate and the CPI-stack identity stops closing.  Hence sets, not one giant list.
#
#   ./perf-smol.sh cpi  ./prog      CPI stack: every stall cause + frontend breakdown  (10)
#   ./perf-smol.sh mem  ./prog      D$/I$ traffic and loads/stores                     ( 6)
#   ./perf-smol.sh br   ./prog      redirect causes                                    ( 4)
#   ./perf-smol.sh all  ./prog      everything -- MULTIPLEXED, estimates only          (18)
#
# Pipe the output through tools/perf-cpi-stack.py.
set -u
SET=${1:-cpi}; shift || true

case "$SET" in
  # ST_MEM..ST_SER + FE_MMU/FE_IC/FE_ALN/FE_QUE.  FE_BUB (r0310) is deliberately NOT here:
  # it is the sum of the four FE_* causes, so the parser reconstructs it and we spend the
  # counter on something we cannot derive.
  cpi) EV=r0300,r0301,r0302,r0303,r0304,r0311,r0312,r0313,r0314 ;;
  mem) EV=r0003,r0004,r0100,r0102,r0110,r0112 ;;
  br)  EV=r0005,r0006,r0007,r0008 ;;
  all) EV=r0003,r0004,r0005,r0006,r0007,r0008,r0100,r0102,r0110,r0112,r0300,r0301,r0302,r0303,r0304,r0310,r0311,r0312,r0313,r0314
       echo "warning: 20 raw events > 13 counters -- perf will multiplex and every count is" >&2
       echo "         a scaled ESTIMATE.  The CPI-stack residual will not close.  Prefer 'cpi'." >&2 ;;
  *)   echo "usage: $0 {cpi|mem|br|all} COMMAND..." >&2; exit 2 ;;
esac

# No sudo if the admin lowered perf_event_paranoid (see tools/perf-smol-setup.sh); fall back
# to sudo only when the kernel actually refuses.
SUDO=""
[ "$(cat /proc/sys/kernel/perf_event_paranoid 2>/dev/null || echo 2)" -gt 1 ] && SUDO="sudo"
exec $SUDO perf stat -e "cycles,instructions,$EV" "$@"
