#!/usr/bin/env bash
# perf-smol -- count or sample the SmolRV64 core's performance events, on the board.
#
#   perf-smol SET CMD [ARGS...]       count CMD's events (perf stat); the report goes to stderr
#   perf-smol rec EVENTS CMD [ARGS]   sample CMD (perf record): which functions the events hit
#   perf-smol events                  every event's name, raw code and meaning
#   perf-smol                         this help
#
# Examples
#   perf-smol top pi 100000 > /dev/null     # the program's output dropped, the report shown
#   perf-smol memcpi ./prog                 # the memory events, under the top-down
#   perf-smol rec td ./prog > /dev/null     # which functions lose the cycles, and to what
#   perf-smol rec DCMISS,RED_BR ./prog      # which functions miss the D$ and mispredict
#
# COUNTING. Every set counts the top-down: the sixteen causes outside dispatching, each on its own
# fixed counter (16..31), with cycles and instructions. The report prints level 1 -- dispatching,
# bad speculation, front-end, back-end, closing to 100% of the cycles -- then the causes under
# each, and then the set's own events, which take the 13 general counters:
#   top     only the top-down, with the dispatch width (1/2/3/4 a cycle), the slots,
#           and redirects and misses per 1000 instructions                            (10)
#   cpi     the wait events, which overlap: every ST_* cause, the front-end
#           breakdown, ROB-full, the redirect drain, D$ misses                        (13)
#   hold    the dispatch hold (ST_DSP) by cause: the head's scheduler, rename,
#           the store and load queues, serialising                                    ( 8)
#   mem     D$ and I$ traffic, loads and stores, the dMMU walker                      ( 8)
#   memcpi  the memory events, the queue occupancies, the fast-load latency           (13)
#   br      the redirects by cause, and the drain                                     ( 5)
# (n) is the general counters a set uses. More than 13 would make perf multiplex, and every count
# would be an estimate, so no set asks for more. The counts stay in perf-smol.SET.txt;
# `perf-cpi-stack.py perf-smol.SET.txt` prints the report again. SmolRV64-Spec 11, "Reading the
# report", says what each line points at.
#
# SAMPLING. EVENTS is `td` (the sixteen top-down causes) or event names or raw codes,
# comma-separated: TDK_BE_DEPLD,DCMISS or r0102. cycles is sampled too. The report lists the
# hottest functions: for `td`, each one's share of the cycles and of the stall cycles, and the
# causes that take its own cycles; for named events, each one's share of each event. A sample lands
# up to ~30 cycles after its event, plus the interrupt: trust the function, not the instruction.
# The samples stay in perf-smol.rec.data; `perf report -i perf-smol.rec.data` gives call chains.
#
# ENVIRONMENT
#   PERF_OUT=FILE        where the counts or the samples go
#   PERF_TIMEOUT_MS=N    counting: stop after N ms and still print the counts (perf's own
#                        --timeout; an external `timeout` kills perf before it prints)
#   PERF_FREQ=N          sampling: samples a second over all the events together (1000)
#   PERF_TOP=N           sampling: how many functions the report lists (20)
#
# SETUP. On coffee, tools/perf-smol-install.sh puts perf-smol, perf-cpi-stack.py, perf-smol-rec.py
# and the event table in the boards' shared ~/bin; tools/perf-smol-setup.sh (once per board) lets
# perf run without sudo. The fixed counters are in every bitstream from main d9cbe5bf on, with a
# DTB that maps them; the report stops if they read 0.
set -u
HERE=$(dirname "$(readlink -f "$0")")
usage() { sed -n '2,/^[^#]/{/^#/s/^# \{0,1\}//p}' "$0"; exit "${1:-0}"; }

[ $# -ge 1 ] || usage 0
SET=$1; shift
case "$SET" in
  -h|--help|help) usage 0 ;;
  events) exec "$HERE/perf-smol-rec.py" list ;;
  # the dispatch widths, the redirects, the misses and DPATCH
  top) EV=r0500,r0501,r0502,r0006,r0007,r0008,r0102,r0112,r0104,r0408 ;;
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
  rec) ;;
  *)   echo "perf-smol: no set '$SET'" >&2; usage 2 >&2 ;;
esac
[ $# -ge 1 ] || usage 2 >&2

# No sudo if the admin lowered perf_event_paranoid (see tools/perf-smol-setup.sh); fall back
# to sudo only when the kernel actually refuses.
SUDO=""
[ "$(cat /proc/sys/kernel/perf_event_paranoid 2>/dev/null || echo 2)" -gt 1 ] && SUDO="sudo"

if [ "$SET" = rec ]; then
  [ $# -ge 2 ] || usage 2 >&2
  SPEC=$1; shift
  EV=$("$HERE/perf-smol-rec.py" rec "$SPEC") || exit 2
  OUT=${PERF_OUT:-perf-smol.rec.data}
  # PERF_FREQ is the whole interrupt rate: each event gets its share
  F=$(( ${PERF_FREQ:-1000} / $(tr ',' '\n' <<< "$EV" | wc -l) ))
  [ "$F" -ge 10 ] || F=10
  $SUDO perf record -q -g -F "$F" -o "$OUT" -e "$EV" "$@"
  rc=$?
  $SUDO perf script -i "$OUT" -F event,period,ip,sym,dso --hide-call-graph 2>/dev/null \
    | "$HERE/perf-smol-rec.py" report "$SPEC" >&2 \
    && echo "  (samples in $OUT; perf report -i $OUT for the call chains)" >&2
  exit $rc
fi

# PERF_TIMEOUT_MS: stop after N ms and PRINT.  Use perf's own --timeout, never an external
# `timeout`: SIGTERM kills perf before it emits the counter block, and SIGINT to perf is
# ignored while it waits on the workload -- a three-hour Geekbench slice was lost to exactly
# that, and the log looks merely truncated rather than failed.  The only other reliable
# lever is killing the WORKLOAD, which makes perf print normally.
TD=$("$HERE/perf-smol-rec.py" codes td) || exit 2
OUT=${PERF_OUT:-perf-smol.$SET.txt}
$SUDO perf stat ${PERF_TIMEOUT_MS:+--timeout "$PERF_TIMEOUT_MS"} -o "$OUT" -e "cycles,instructions,$TD,$EV" "$@"
rc=$?
"$HERE/perf-cpi-stack.py" "$OUT" >&2 && echo "  (counts in $OUT)" >&2
exit $rc
