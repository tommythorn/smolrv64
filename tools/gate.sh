#!/usr/bin/env bash
# THE GATE, as one command. Build at the shipping configuration, program, boot, and decide.
#
#   tools/gate.sh              # gate the working tree
#   tools/gate.sh <commit>     # check out <commit> first (detached), then gate it
#
# Two hard gates, in this order (docs/SmolRV64-Spec.md, and Tommy 2026-09-01):
#   1. probe_clk closes at 166.67 MHz -- `make` alone, no last-mile pass to remember
#   2. the board boots to a login: prompt WITH ZERO PROCESSES SEGFAULTING
# Neither is tradeable. An IPC regression on the way to more IPC is fine; a WNS miss is not,
# and a bitstream that closes but does not boot is worth nothing.
#
# Exists because gating a BATCH of commits and only testing the tip cannot attribute a
# breakage: when the D$ split Oopsed the board on 2026-09-01, the cache change and a
# zero-slack bitstream had moved together and the board could not say which was at fault.
# Per-commit costs the same hour as bisecting later, and answers immediately.
set -u
cd "$(dirname "$0")/.."
REPO=$(pwd)
UB=$REPO/workloads/ubuntu
BOOT_WAIT=${BOOT_WAIT:-1800}      # seconds to wait for login:
# Three boards hang off coffee, so the board under test is named (tools/board-gate.sh) before the
# gate spends its netlist boot and stamp on a bitstream it cannot then program; the board names
# the platform the gate builds (tools/boards.sh). NO_BOARD=1 builds the RK board's unless BOARD says.
if [ "${NO_BOARD:-0}" = 0 ] && [ -z "${BOARD:-}" ]; then
   echo "GATE: BOARD=2g|4g|hdmi names the board under test (NO_BOARD=1 stops before the board)"; exit 2
fi
MAIN=$(git -C "$REPO" worktree list | head -1 | awk '{print $1}')
. "$REPO/tools/boards.sh" || { echo "GATE: BOARD=$BOARD: 2g, 4g or hdmi"; exit 2; }
PSRC=$PLAT/$PROJ.srcs            # the board's own design sources, as the build counts them
PLAT=$REPO/$PLAT
# The placer directive the board's builds use, which the stamp's lookup must match (its Makefile).
P=$(make -s -C "$PLAT" print-PLACE_DIRECTIVE 2>/dev/null)
[ -n "$P" ] && export PLACE_DIRECTIVE=${PLACE_DIRECTIVE:-$P}

if [ $# -ge 1 ]; then
   git checkout -q --detach "$1" || { echo "GATE: cannot check out $1"; exit 2; }
fi
WHAT=$(git log -1 --format='%h %s' | cut -c1-70)
echo "=== GATE: $WHAT ==="

# Evidence outlives the build tree. `make clean` deletes *.log inside the platform dir, so a
# log written there is gone the moment the NEXT gate runs -- which is how the 2026-09-02 pass
# lost its timing report and with it the answer to "where is the margin now". Results go
# under the main checkout's root, keyed by commit, where nothing in the platform Makefile
# reaches them and a worktree's deletion does not take them.
# A tree with uncommitted changes gets its own directory, named by its content as well: two
# worktrees at one commit would otherwise bank their bitstreams over each other.
SHA=$(git rev-parse --short HEAD)
DIRTY=$(git status --porcelain --untracked-files=no -- src core platforms/xcku5p "$PSRC" workloads/ubuntu workloads/tiny128 workloads/monitor)
# A board with another platform builds another bitstream from the same commit: its own directory.
RES="$(cd "$(git rev-parse --git-common-dir)/.." && pwd)/gate-results/$SHA${DIRTY:+-c$("$REPO"/tools/bitcache.sh content)}"
[ "$PROJ" = rk_xcku5p ] || RES=$RES-$PROJ
mkdir -p "$RES"
git log -1 --format='%H%n%s%n%ci' > "$RES/commit.txt"

# ---- 1. fast gates first: they cost seconds and can veto an hour ----------------------
(cd "$REPO/src" && ./lint.sh) > "$RES/lint.log" 2>&1 || { tail -5 "$RES/lint.log"; echo "GATE: FAIL (lint)"; exit 1; }
tail -1 "$RES/lint.log"
# EVERY unit tb, found by GLOB rather than named one at a time. Only the cache tb was ever
# run here; on 2026-09-03 three of the other four turned out not even to COMPILE -- tb_smolrv64_iq
# lost `iss_ps` when the tags moved to a LUTRAM, and tb_smolrv64_lq/tb_smolrv64_sq still drive the
# per-candidate address ports that the conflict-matrix rewrite deleted. Nothing noticed,
# because no gate ran them and the runners sent their build errors to /dev/null. A test that
# does not compile is a test that cannot fail, which is the same defect as a test that passes
# vacuously. A build failure here is a GATE failure, and a new run-*-tb.sh is picked up
# without editing this file. src/run-tb.sh runs every src/ bench (the SoC devices -- the scanout,
# virtio, the DDR bridge -- and the blocks the core shares) the same way.
# The benches are simulations, and simulations run on the simulation host (Tommy, 2026-10-15:
# none on coffee). GATE_TB_LOGS=<dir> names their results, produced there from these sources:
# content.txt (tools/bitcache.sh content: every input the benches read) and each bench's <name>.log
# and <name>.rc; the verdict is the same.
if [ -n "${GATE_TB_LOGS:-}" ] && [ "$(cat "$GATE_TB_LOGS/content.txt" 2>/dev/null)" != "$("$REPO"/tools/bitcache.sh content)" ]; then
   echo "GATE: FAIL (GATE_TB_LOGS holds other sources' benches)"; exit 1
fi
for tb in "$REPO"/core/run-*-tb.sh "$REPO"/src/run-tb.sh; do
   [ -x "$tb" ] || continue
   n=$(basename "$tb" .sh)
   if [ -n "${GATE_TB_LOGS:-}" ]; then
      cp "$GATE_TB_LOGS/$n.log" "$RES/$n.log" 2>/dev/null || { echo "GATE: FAIL ($n: no log in GATE_TB_LOGS)"; exit 1; }
      rc=$(cat "$GATE_TB_LOGS/$n.rc" 2>/dev/null || echo 1)
   else
      (cd "$REPO/core" && "$tb") > "$RES/$n.log" 2>&1
      rc=$?
   fi
   grep -E 'PASS|FAIL|BUILD FAILED' "$RES/$n.log" | sed "s/^/  [$n] /"
   if [ "$rc" != 0 ] || grep -q 'FAIL' "$RES/$n.log"; then
      echo "GATE: FAIL ($n)"; exit 1
   fi
done

# ---- 2. the floor: `make` alone, from a clean tree ------------------------------------
# The post-synthesis netlist boots the monitor before an hour of Vivado is spent (rule F5):
# four two-wide bitstreams were silent on the board while every simulation passed, and the
# netlist wedged inside 1000 cycles (2026-09-06).
echo "--- netlist boot (rule F5) ---"
"$REPO"/tools/netlist-boot.sh > "$RES/netlist-boot.log" 2>&1
NB=$(grep -a "^NETLIST-BOOT:" "$RES/netlist-boot.log" | tail -1)
echo "$NB"
if ! echo "$NB" | grep -q "NETLIST-BOOT: PASS"; then
   echo "GATE: FAIL (netlist)   [evidence: $RES/netlist-boot.log]"; exit 1
fi
# THE IDENTITY STAMP. The build's identity is INIT bits alone (docs/rtl-rules.md I2): a met probe
# of exactly these sources at the shipping configuration IS this gate's design, placed and routed,
# and becomes this commit's bitstream once the commit's identity is stamped into its routed
# checkpoint (tools/stamp-identity.tcl) -- minutes instead of a build. Only a clean tree is
# stamped (its identity: dirty 0, the commit's date, as build.tcl gives it); NO_STAMP=1 builds.
BIT="$PLAT/$PROJ.runs/impl_1/$PROJ.bit"
SRC=""
if [ "${NO_STAMP:-0}" = 0 ] && [ -z "$(git status --porcelain --untracked-files=no -- src core platforms/xcku5p "$PSRC" workloads/ubuntu workloads/tiny128 workloads/monitor)" ]; then
   SRC=$("$REPO"/tools/bitcache.sh stampable "$("$REPO"/tools/bitcache.sh content)" "$PLAT") || SRC=""
fi
if [ -n "$SRC" ]; then
   echo "--- stamping $(basename "$SRC") (these sources, met at WNS $(awk '/^wns:/ {print $2}' "$SRC/meta.txt")) ---"
   XIL=${XIL:-/home/tommy/Xilinx/2025.1/Vivado}
   ( export LD_LIBRARY_PATH=$XIL/lib/lnx64.o/Ubuntu/24
     "$XIL/bin/vivado" -mode batch -nojournal -nolog -source "$REPO/tools/stamp-identity.tcl" -tclargs \
        "$SRC/routed.dcp" "$(git rev-parse --short=8 HEAD)" 0 \
        "$(git log -1 --format=%cd --date=format:%Y%m%d%H%M%S HEAD)" "$RES/stamp" "$PROJ" ) > "$RES/build.log" 2>&1
   # the stamp's own output, never a bitstream an earlier build left in the platform directory
   BIT="$RES/stamp/$PROJ.bit"
   if grep -q "^STAMP-DONE" "$RES/build.log"; then
      cat "$RES/stamp/identity.txt"
      echo "stamped from: $SRC" >> "$RES/commit.txt"
      echo "Timing met: $(grep -oE 'WNS=[-0-9.]+' "$RES/build.log" | tail -1)" >> "$RES/build.log"
   elif grep -q "^STAMP-REFUSED" "$RES/build.log"; then
      # the identical netlist would fail again: a timing failure, no build (BIT stays absent)
      echo "stamped from: $SRC (refused)" >> "$RES/commit.txt"
      echo "TIMING VIOLATION: $(grep -oE 'WNS=[-0-9.]+' "$RES/build.log" | tail -1)" >> "$RES/build.log"
   else
      echo "stamp failed ($(grep -E '^ERROR|stamp-identity:' "$RES/build.log" | head -1 | cut -c1-120)): building"
      SRC=""; BIT="$PLAT/$PROJ.runs/impl_1/$PROJ.bit"
   fi
fi
if [ -z "$SRC" ]; then
   echo "--- building (no arguments; the shipping config is the default) ---"
   ( cd "$PLAT" && git clean -fxdq . && timeout 10800 make ) > "$RES/build.log" 2>&1
   # A build that misses gets the last-mile post-route phys_opt (physopt_close.tcl): a closed
   # result is banked by `make physopt` and gated like any met build.
   if grep -q '^\*\*\* TIMING VIOLATION' "$RES/build.log"; then
      echo "--- last-mile physopt ---"
      ( cd "$PLAT" && timeout 7200 make physopt ) >> "$RES/build.log" 2>&1
      w=$(sed -n 's/^PHYSOPT-CLOSE: BITSTREAM WRITTEN (WNS=\([-0-9.]*\)).*/\1/p' "$RES/build.log" | tail -1)
      [ -n "$w" ] && echo "Timing met: WNS=${w}ns (make physopt)" >> "$RES/build.log"
   fi
fi
WNS=$(grep -oE "Timing met: WNS=[-0-9.]+|TIMING VIOLATION: WNS=[-0-9.]+" "$RES/build.log" | grep -v '^#' | tail -1)
echo "$WNS" > "$RES/wns.txt"
# The worst path is worth keeping WHETHER OR NOT it met: a pass at +0.019 ns names the path
# the next change has to avoid, and that is the whole input to headroom work.
if [ -n "$SRC" ]; then R="$RES/stamp/timing_summary.rpt"
else R=$(ls -t "$PLAT"/$PROJ.runs/impl_1/*timing_summary*.rpt 2>/dev/null | head -1); fi
[ -n "$R" ] && cp "$R" "$RES/timing_summary.rpt"
# The first VIOLATED path, else the first path in the core clock's group: the first "Slack"
# in the report is a debug-hub path with 11 ns to spare, which is what this printed for
# every failed gate until 2026-09-05.
WORST=$(awk '/^Slack \(VIOLATED\)/{sl=$4+0; blk=$0; p=1; next}
             p&&/Source:|Destination:|Data Path Delay|Logic Levels/{blk=blk"\n"$0}
             p&&/Logic Levels/{p=0; if(sl<best){best=sl; keep=blk}}
             END{if(keep!="")print keep}' best=0 "$RES/timing_summary.rpt" 2>/dev/null)
[ -z "$WORST" ] && WORST=$(awk '/^Slack \(MET\)/{blk=$0; n=0; p=1; next} p{blk=blk"\n"$0; if(/Path Group:/){ if($3=="probe_clk"){print blk; exit} else p=0}}' \
                              "$RES/timing_summary.rpt" 2>/dev/null | grep -E "Slack|Source:|Destination:")
[ -n "$WORST" ] && printf '%s\n' "$WORST" > "$RES/worst-path.txt"
if [ ! -f "$BIT" ]; then
   # Name the stage that failed: a build that never reached timing is not a timing failure
   # (gate U, 2026-09-05: two "FAIL (timing)" verdicts were the RAM-inference check, and
   # one of them was misread as memory pressure).
   if grep -q 'RAM INFERENCE REGRESSED' "$RES/build.log"; then WHY="ram-inference: $(grep -oE '^ +[a-z_0-9]+ +is no longer a RAM' "$RES/build.log" | awk '{print $1}' | tr '\n' ' ')"
   elif [ -z "$WNS" ]; then WHY="build: $(grep -E '^ERROR:|^Error|^make: ' "$RES/build.log" | head -1 | cut -c1-120)"
   else WHY="timing"; fi
   echo "GATE: FAIL ($WHY) $WNS   [evidence: $RES/]"
   printf '%s\n' "$WORST"
   exit 1
fi
echo "timing: $WNS"
printf '%s\n' "$WORST" | sed 's/^/  /'
# The bitstream is banked in the evidence directory, and that copy is the one the board gate
# programs (tools/board-gate.sh <resultdir>).
cp "$BIT" "$RES/$PROJ.bit"
# NO_BOARD=1: stop here -- for a build whose board is busy (a Geekbench run on the last gate's
# bitstream); tools/board-gate.sh on the evidence directory finishes the gate later.
if [ "${NO_BOARD:-0}" != 0 ]; then
   echo "GATE: BUILT (board pending)  ($WHAT)  $WNS   [evidence: $RES/, bitstream banked there]"; exit 0
fi

# ---- 3. the board: tools/board-gate.sh, runnable alone on a banked bitstream ------------
tools/board-gate.sh "$RES" | tee "$RES/board.log"
if [ "${PIPESTATUS[0]}" -eq 0 ]; then
   echo "GATE: PASS  ($WHAT)  $WNS   [evidence: $RES/]"; exit 0
fi
echo "GATE: FAIL (board)   [evidence: $RES/]"; exit 1
