#!/usr/bin/env bash
# Run a command under perf with a smolrv64 event set that FITS IN THE COUNTERS, and print the
# report (tools/perf-cpi-stack.py) on stderr when it ends. The raw counts stay in
# perf-smol.<set>.txt (PERF_OUT=<file> to choose), which perf-cpi-stack.py re-reads.
#
# On the board: tools/perf-smol-install.sh (on coffee) installs this as ~/bin/perf-smol, with the
# report script and the event table beside it, into the boards' shared home. Then
#   perf-smol td pi 100000 > /dev/null        # the program's output dropped, the report shown
#
# The DTB exposes 13 general counters (riscv,raw-event-to-mhpmcounters ... 0x0000fff8 =
# mhpmcounter3..15), any event each, and 16 fixed ones (16..31), each counting one top-down
# cause (td_k 3-8, 10, 12-20) and only that.  cycles and instructions ride the dedicated
# mcycle/minstret and are free.  Ask for more than 13 general events and perf MULTIPLEXES:
# every count becomes a scaled estimate and the CPI-stack identity stops closing.  Hence sets.
#
#   perf-smol top  ./prog      TOP-DOWN, every level in one run: each td_k cause on its own
#                              counter, the dispatch width, slots, redirects and misses    (16+10)
#   perf-smol td   ./prog      TOP-DOWN: level 1 closes to 100% of cycles, depth under each
#                              parent, slots, redirects and misses per 1k                  (13)
#   perf-smol tdbe ./prog      TOP-DOWN level 3: the back-end's ten causes (td_k), which
#                              sum to TD_BE                                                (13)
#   perf-smol tdfe ./prog      TOP-DOWN level 3: bad speculation's and the front-end's
#                              causes and the dispatch width                               (13)
#   perf-smol cpi  ./prog      the wait events (overlapping): every ST_* cause, the frontend
#                              breakdown, ROB-full, the redirect drain, D$ misses          (13)
#   perf-smol mem  ./prog      D$/I$ traffic, loads/stores, the dMMU walker                ( 8)
#   perf-smol memcpi ./prog    the memory events, queue occupancies, fast-load latency     (13)
#   perf-smol br   ./prog      redirect causes + the drain                                 ( 5)
#   perf-smol hold ./prog      the dispatch hold (ST_DSP) by cause: the head's scheduler,
#                              rename, the store and load queues, serialising              ( 8)
#   perf-smol all  ./prog      24 of the events -- MULTIPLEXED, estimates only             (24)
#
# Running only the FE_* events (r0310-r0314) names no back-end wait: every one lands in
# "unattributed".  Use 'cpi', or the td sets.
#
# PERF_TIMEOUT_MS=10800000 perf-smol cpi ./long-workload   # 3 h slice, counters printed
set -u
SET=${1:-td}; shift || true

case "$SET" in
  # TOP-DOWN (SmolRV64-Spec 11.2): TD_BS + TD_FE + TD_BE are exclusive and dispatching is the rest,
  # so level 1 closes by construction; TD_BE_MEM/ROB/IQ and TD_FE_LAT are subsets of their
  # parent, RD_WAIT splits bad speculation, DPATCH gives the slots. 13, the whole counter file.
  td)  EV=r0401,r0407,r0402,r0408,r0403,r0404,r0405,r0406,r0317,r0006,r0007,r0102,r0112 ;;
  # td_k itself (the TDK_* events, 0x05nn = cycles with td_k == nn): level 1 again, and the
  # back-end's ten causes, which the report checks sum to TD_BE.
  tdbe) EV=r0401,r0407,r0402,r050a,r050c,r050d,r050e,r050f,r0510,r0511,r0512,r0513,r0514 ;;
  # ALL of td_k in one session: the sixteen causes outside dispatching have a counter each
  # (16..31, SmolRV64-Spec 11), dispatched 1/2/3 take general counters (4 is the rest), and the
  # general counters left hold the redirects, the misses and DPATCH. 26 events, no multiplexing.
  top) EV=r0503,r0504,r0505,r0506,r0507,r0508,r050a,r050c,r050d,r050e,r050f,r0510,r0511,r0512,r0513,r0514,r0500,r0501,r0502,r0006,r0007,r0008,r0102,r0112,r0104,r0408 ;;
  # ...and bad speculation's two causes, the front-end's four and dispatched 1/2/3, with DPATCH.
  tdfe) EV=r0401,r0407,r0402,r0503,r0504,r0505,r0506,r0507,r0508,r0500,r0501,r0502,r0408 ;;
  # ST_MEM..ST_ROB + FE_MMU/FE_IC/FE_ALN/FE_QUE + RD_WAIT + REDIR + DCMISS = 13, the whole
  # counter file.  FE_BUB (r0310) is not here: it is the sum of the four FE_* causes, so the
  # report reconstructs it and the counter goes to something it cannot derive.
  cpi) EV=r0300,r0301,r0302,r0303,r0304,r0305,r0311,r0312,r0313,r0314,r0317,r0005,r0102 ;;
  # + the dMMU walker: its starts (r0104) and its busy cycles (r0318).
  mem) EV=r0003,r0004,r0100,r0102,r0110,r0112,r0104,r0318 ;;
  # The memory events: the port, the door, the alias blocks, the reorders, the dropped landings,
  # the device waits, the queue occupancies, and fast loads in flight with their starts (their
  # ratio is the mean fast-load latency), with loads and misses beside.
  memcpi) EV=r0319,r031a,r031b,r031c,r031d,r031e,r031f,r0320,r0321,r0322,r0323,r0003,r0102 ;;
  br)  EV=r0005,r0006,r0007,r0008,r0317 ;;
  # The dispatch hold (ST_DSP) broken down: scheduler / rename / store queue / load queue /
  # serializing, with the ROB and the redirects beside them.
  hold) EV=r0304,r0305,r0306,r0307,r0308,r0309,r030a,r0005 ;;
  all) EV=r0003,r0004,r0005,r0006,r0007,r0008,r0100,r0102,r0104,r0110,r0112,r0300,r0301,r0302,r0303,r0304,r0305,r0310,r0311,r0312,r0313,r0314,r0317,r0318
       echo "warning: 24 raw events > 13 counters -- perf will multiplex and every count is" >&2
       echo "         a scaled ESTIMATE.  The CPI-stack residual will not close.  Prefer 'cpi'." >&2 ;;
  *)   echo "usage: $0 {top|td|tdbe|tdfe|cpi|hold|mem|memcpi|br|all} COMMAND..." >&2; exit 2 ;;
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
OUT=${PERF_OUT:-perf-smol.$SET.txt}
$SUDO perf stat ${PERF_TIMEOUT_MS:+--timeout "$PERF_TIMEOUT_MS"} -o "$OUT" -e "cycles,instructions,$EV" "$@"
rc=$?
HERE=$(dirname "$(readlink -f "$0")")
"$HERE/perf-cpi-stack.py" "$OUT" >&2 && echo "  (counts in $OUT)" >&2
exit $rc
