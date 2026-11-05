#!/usr/bin/env bash
# The bitstream cache: every bitstream `make bit` produces, kept under $BITCACHE (~/bitcache) by
# what it was built from, so a board test or a bisection programs a cached bitstream instead of
# rebuilding one.
#
#   tools/bitcache.sh put [<platform dir>]   bank impl_1's bitstream (make bit does this)
#   tools/bitcache.sh get <commit-prefix> [<knob filter>]   print the newest matching .bit
#   tools/bitcache.sh list [<commit-prefix>]  the entries, newest first, with WNS
#   tools/bitcache.sh start [<platform dir>]  record the tree a build starts from (make bit does this)
#   tools/bitcache.sh key [<platform dir>]    the key the tree would be banked under
#   tools/bitcache.sh content                 the content id of the tree's build inputs
#   tools/bitcache.sh stampable <content id> [<platform dir>]  the newest met entry of that board's
#                                             the gate may stamp (tools/gate.sh)
#
# A key is <commit12>[-d<diff8>]-iw<IW>-div<DIV8>-<placer>-<project>: the commit, a hash of the tree's
# uncommitted changes to anything synthesized or used to build (src, core, the platform's sources
# and scripts, the monitor), and the knobs the build used (the project file's defines, the
# placer directive in impl_1's log), and the board's project, since every board builds its own
# bitstream from one commit. "Met" is setup and hold both. An entry holds the bitstream, the routed checkpoint (a probe
# build that differs by a knob or an ILA places incrementally against it: INCR_DCP=<entry>/routed.dcp
# make), the routed timing summary, the build's defines, the diff of a dirty tree and meta.txt.
set -eu
BITCACHE=${BITCACHE:-$HOME/bitcache}
here=$(cd "$(dirname "$0")/.." && pwd)
plat_default=$here/platforms/rk-xcku5p-f-v1.2

diff_paths=(src core platforms workloads/monitor)
excl=(':(exclude)*.xpr' ':(exclude)third_party' ':(exclude)*.log' ':(exclude)src/mem.linehex')
# Untracked files no build reads: logs, images, lock files, and what a Vivado run (an
# out-of-context build, a report) writes into its working directory: clockInfo.txt, and the
# placer's congestion estimates under cong/.
junk='\.(log|linehex|elf|bin|flock)$|(^|/)clockInfo\.txt$|(^|/)cong/'

# A define the last build used: the project file's Verilog_Define block records them (build.tcl).
define() {
    grep -o "Verilog_Define Name=\"$2\" Val=\"[^\"]*\"" "$1/$PROJ.xpr" 2>/dev/null | head -1 | sed 's/.*Val="//; s/"$//'
}

# The tree: the commit and, when anything synthesized or used to build differs from it, a hash
# of the difference.
tree_id() {
    local commit d
    commit=$(git -C "$here" rev-parse --short=12 HEAD)
    d=$( (cd "$here" && git diff HEAD -- "${diff_paths[@]}" "${excl[@]}" 2>/dev/null
          git ls-files -o --exclude-standard -- "${diff_paths[@]}" 2>/dev/null | grep -v -E "$junk" | LC_ALL=C sort | xargs -r cat) | sha1sum | cut -c1-8)
    if [ -n "$(cd "$here" && git status --porcelain -- "${diff_paths[@]}" "${excl[@]}" 2>/dev/null | grep -v -E "$junk")" ]
    then echo "$commit-d$d"; else echo "$commit"; fi
}

# The CONTENT of the build's inputs, with no reference to a commit: every file under the build
# paths and third_party (tracked, and untracked but not ignored), by path and git blob hash, as the
# working tree holds them. A dirty probe and the commit made of it share it; the gate's identity
# stamp (tools/gate.sh) relies on exactly that. The list is sorted by bytes (LC_ALL=C): a locale's
# collation orders it differently on coffee (GNU, en_US) and on M5 (BSD), and so did the id.
# core/cosim-expected.txt is left out: no build and no bench reads it (the lockstep runner does),
# and its rows are measured on the finished RTL, so counting it made every landing that moves a
# row a second probe.
content_id() {
    local list
    local cexcl=(':(exclude)*.xpr' ':(exclude)*.log' ':(exclude)src/mem.linehex' ':(exclude)core/cosim-expected.txt')
    list=$(cd "$here" && { git ls-files -c -- "${diff_paths[@]}" third_party "${cexcl[@]}"
                           git ls-files -o --exclude-standard -- "${diff_paths[@]}" | grep -v -E "$junk"
                         } | LC_ALL=C sort -u | while read -r f; do [ -f "$f" ] && echo "$f"; done)
    (cd "$here" && paste <(echo "$list") <(echo "$list" | git hash-object --stdin-paths)) | sha1sum | cut -c1-12
}

# The full key: the tree as `start` recorded it when the build began (else as it is now), and
# the knobs the build used.
key() {
    local plat=$1 tree iw div plc
    tree=$(cat "$plat/$PROJ.runs/bitcache-start/tree" 2>/dev/null || tree_id)
    iw=$(define "$plat" SMOLRV64_IW);  div=$(define "$plat" PROBE_CLK_DIV8)
    # the LAST place_design: runme.log keeps the runs a reset_run left before this one's
    plc=$(grep -a -o 'place_design -directive [A-Za-z_]*' "$plat/$PROJ.runs/impl_1/runme.log" 2>/dev/null | tail -1 | awk '{print $3}')
    echo "$tree-iw${iw:-?}-div${div:-?}-${plc:-?}-$PROJ"
}

# A platform directory's project: its one .xpr.
proj_of() {
    local x; x=$(ls "$1"/*.xpr 2>/dev/null)
    [ "$(echo "$x" | grep -c .)" = 1 ] || { echo "bitcache: $1 has no one .xpr" >&2; exit 1; }
    basename "$x" .xpr
}

cmd=${1:-list}; shift || true
case "$cmd" in key|start|put) PROJ=$(proj_of "${1:-$plat_default}");; esac
case "$cmd" in
key)
    plat=${1:-$plat_default}
    key "$plat"
    ;;
start)
    # what the build is made from, before it starts: a tree edited or committed meanwhile does
    # not change the key or the diff the bitstream is banked with
    plat=${1:-$plat_default}
    mkdir -p "$plat/$PROJ.runs/bitcache-start"
    tree_id > "$plat/$PROJ.runs/bitcache-start/tree"
    content_id > "$plat/$PROJ.runs/bitcache-start/cid"
    (cd "$here" && git diff HEAD -- "${diff_paths[@]}" "${excl[@]}") > "$plat/$PROJ.runs/bitcache-start/dirty.diff" 2>/dev/null || true
    (cd "$here" && git log -1 --format='%H %s') > "$plat/$PROJ.runs/bitcache-start/commit"
    ;;
put)
    plat=${1:-$plat_default}
    bit=$plat/$PROJ.runs/impl_1/$PROJ.bit
    [ -f "$bit" ] || { echo "bitcache: no bitstream at $bit" >&2; exit 1; }
    rpt=$plat/$PROJ.runs/impl_1/${PROJ}_timing_summary_closed.rpt       # make physopt's, when it closed
    [ -f "$rpt" ] || rpt=$plat/$PROJ.runs/impl_1/${PROJ}_timing_summary_postroute_physopted.rpt
    [ -f "$rpt" ] || rpt=$plat/$PROJ.runs/impl_1/${PROJ}_timing_summary_routed.rpt
    k=$(key "$plat")
    d=$BITCACHE/$k
    prev=""
    [ -e "$d" ] && { prev=$d; d=$d.$(date +%Y%m%d%H%M%S); }   # a rebuild of the same key keeps both
    mkdir -p "$d"
    cp "$bit" "$d/$PROJ.bit"
    [ -f "$rpt" ] && cp "$rpt" "$d/timing_summary.rpt"
    dcp=$plat/$PROJ.runs/impl_1/${PROJ}_closed.dcp
    [ -f "$dcp" ] || dcp=$plat/$PROJ.runs/impl_1/${PROJ}_postroute_physopt.dcp
    [ -f "$dcp" ] || dcp=$plat/$PROJ.runs/impl_1/${PROJ}_routed.dcp
    [ -f "$dcp" ] && cp "$dcp" "$d/routed.dcp"
    grep -o 'Verilog_Define Name="[^"]*" Val="[^"]*"' "$plat/$PROJ.xpr" | sed 's/Verilog_Define Name="//; s/" Val="/=/; s/"$//; s/&apos;/'"'"'/g; s/&quot;/"/g' > "$d/defines.txt" || true
    st=$plat/$PROJ.runs/bitcache-start
    if [ -f "$st/tree" ]; then cp "$st/dirty.diff" "$d/dirty.diff"; commit=$(cat "$st/commit")
    else (cd "$here" && git diff HEAD -- "${diff_paths[@]}" "${excl[@]}") > "$d/dirty.diff" 2>/dev/null || true
         commit=$(cd "$here" && git log -1 --format='%H %s'); fi
    [ -s "$d/dirty.diff" ] || rm -f "$d/dirty.diff"
    wns=$(grep -A2 'WNS(ns)' "$d/timing_summary.rpt" 2>/dev/null | sed -n 3p | awk '{print $1}')
    whs=$(grep -A2 'WNS(ns)' "$d/timing_summary.rpt" 2>/dev/null | sed -n 3p | awk '{print $5}')
    { echo "key:     $k"
      echo "project: $PROJ"
      echo "commit:  $commit"
      echo "tree:    $here"
      echo "built:   $(date '+%Y-%m-%d %H:%M:%S')"
      # a re-bank of a key (make physopt's second pass) has no start record left: the key's first
      # entry was built from the same sources
      echo "cid:     $(cat "$st/cid" 2>/dev/null || sed -n 's/^cid: *//p' "$prev/meta.txt" 2>/dev/null | grep . || echo ?)"
      echo "wns:     ${wns:-?}"
      echo "whs:     ${whs:-?}"
      echo "met:     $(awk -v w="${wns:-x}" -v h="${whs:-x}" 'BEGIN{print (w ~ /^-?[0-9.]+$/ && w >= 0 && h ~ /^-?[0-9.]+$/ && h >= 0) ? "yes" : "NO"}')"
    } > "$d/meta.txt"
    rm -rf "$st"
    echo "bitcache: banked $d (WNS ${wns:-?})"
    ;;
content)
    content_id
    ;;
stampable)
    # A met entry built from these sources at the shipping configuration: the default knobs and no
    # define but the build's own (the identity, the monitor's path), so its netlist is the gate's
    # but for the identity's INIT bits. It must hold the routed checkpoint the stamp starts from.
    # Placed with the directive the gate's own build would use (PLACE_DIRECTIVE, as build.tcl reads
    # it): a directive moves placement, not the netlist, and a met build is met.
    cid=${1:?content id}
    PROJ=$(proj_of "${2:-$plat_default}")
    pl=${PLACE_DIRECTIVE:-AltSpreadLogic_medium}
    iw=$(sed -n 's/^ *`define SMOLRV64_IW \([0-9]*\).*/\1/p' "$here/core/smolrv64_width.vh")   # the shipping width
    for d in $(ls -td "$BITCACHE"/*-iw$iw-div48-"$pl"-"$PROJ" "$BITCACHE"/*-iw$iw-div48-"$pl"-"$PROJ".* 2>/dev/null); do
        grep -q "^cid: *$cid\$" "$d/meta.txt" 2>/dev/null || continue
        grep -q '^met: *yes' "$d/meta.txt" && [ -f "$d/routed.dcp" ] || continue
        extra=$(sed 's/=.*//' "$d/defines.txt" | grep -v -x -E 'MEM_BASEADDR|SOC_BOOT_HEX|PROBE_CLK_DIV8|SMOLRV64_HW|SMOLRV64_IW|SMOLRV64_GIT_COMMIT|SMOLRV64_GIT_DIRTY|SMOLRV64_BUILD_STAMP' || true)
        [ -z "$extra" ] && grep -q -x 'PROBE_CLK_DIV8=48' "$d/defines.txt" && grep -q -x 'SMOLRV64_HW=8' "$d/defines.txt" \
            && grep -q -x "SMOLRV64_IW=$iw" "$d/defines.txt" || continue
        echo "$d"; exit 0
    done
    exit 1
    ;;
get)
    pre=${1:?commit prefix}; filt=${2:-}
    for d in $(ls -td "$BITCACHE"/"$pre"* 2>/dev/null); do
        case "$d" in *"$filt"*) grep -q '^met: *yes' "$d/meta.txt" 2>/dev/null && { ls "$d"/*.bit | head -1; exit 0; };; esac
    done
    echo "bitcache: no timing-clean bitstream for $pre${filt:+ ($filt)}" >&2; exit 1
    ;;
list)
    pre=${1:-}
    for d in $(ls -td "$BITCACHE"/"$pre"* 2>/dev/null); do
        printf '%-60s WNS %-8s %s\n' "$(basename "$d")" "$(awk '/^wns:/ {print $2}' "$d/meta.txt")" \
            "$(awk '/^commit:/ {$1=""; $2=""; print substr($0, 3, 60)}' "$d/meta.txt")"
    done
    ;;
*)  sed -n '2,15p' "$0"; exit 1 ;;
esac
