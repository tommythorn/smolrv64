#!/usr/bin/env bash
# The bitstream cache: every bitstream `make bit` produces, kept under $BITCACHE (~/bitcache) by
# what it was built from, so a board test or a bisection programs a cached bitstream instead of
# rebuilding one.
#
#   tools/bitcache.sh put [<platform dir>]   bank impl_1's bitstream (make bit does this)
#   tools/bitcache.sh get <commit-prefix> [<knob filter>]   print the newest matching .bit
#   tools/bitcache.sh list [<commit-prefix>]  the entries, newest first, with WNS
#   tools/bitcache.sh key [<platform dir>]    the key the tree would be banked under
#
# A key is <commit12>[-d<diff8>]-iw<IW>-div<DIV8>-<placer>: the commit, a hash of the tree's
# uncommitted changes to anything synthesized or used to build (src, core, the platform's sources
# and scripts, the monitor), and the knobs the build used (the project file's defines, the
# placer directive in impl_1's log). An entry holds the
# bitstream, the routed timing summary, the build's defines, the diff of a dirty tree and meta.txt.
set -eu
BITCACHE=${BITCACHE:-$HOME/bitcache}
here=$(cd "$(dirname "$0")/.." && pwd)
plat_default=$here/platforms/rk-xcku5p-f-v1.2

diff_paths=(src core platforms/rk-xcku5p-f-v1.2 workloads/monitor)
excl=(':(exclude)*.xpr' ':(exclude)third_party' ':(exclude)*.log' ':(exclude)src/mem.linehex')

# A define the last build used: the project file's Verilog_Define block records them (build.tcl).
define() {
    grep -o "Verilog_Define Name=\"$2\" Val=\"[^\"]*\"" "$1/$PROJ.xpr" 2>/dev/null | head -1 | sed 's/.*Val="//; s/"$//'
}

key() {
    local plat=$1
    local commit dirty="" iw div plc
    commit=$(git -C "$here" rev-parse --short=12 HEAD)
    local d
    d=$( (cd "$here" && git diff HEAD -- "${diff_paths[@]}" "${excl[@]}" 2>/dev/null
          git ls-files -o --exclude-standard -- "${diff_paths[@]}" 2>/dev/null | grep -v -E '\.(log|linehex|elf|bin)$' | sort | xargs -r cat) | sha1sum | cut -c1-8)
    [ -n "$(cd "$here" && git status --porcelain -- "${diff_paths[@]}" "${excl[@]}" 2>/dev/null | grep -v -E '\.(log|linehex|elf|bin)$')" ] && dirty="-d$d"
    iw=$(define "$plat" SMOLRV64_IW);  div=$(define "$plat" PROBE_CLK_DIV8)
    plc=$(grep -a -o -m1 'place_design -directive [A-Za-z_]*' "$plat/$PROJ.runs/impl_1/runme.log" 2>/dev/null | awk '{print $3}')
    echo "$commit$dirty-iw${iw:-?}-div${div:-?}-${plc:-?}"
}

PROJ=rk_xcku5p
cmd=${1:-list}; shift || true
case "$cmd" in
key)
    plat=${1:-$plat_default}
    key "$plat"
    ;;
put)
    plat=${1:-$plat_default}
    bit=$plat/$PROJ.runs/impl_1/$PROJ.bit
    [ -f "$bit" ] || { echo "bitcache: no bitstream at $bit" >&2; exit 1; }
    rpt=$plat/$PROJ.runs/impl_1/${PROJ}_timing_summary_postroute_physopted.rpt
    [ -f "$rpt" ] || rpt=$plat/$PROJ.runs/impl_1/${PROJ}_timing_summary_routed.rpt
    k=$(key "$plat")
    d=$BITCACHE/$k
    [ -e "$d" ] && d=$d.$(date +%Y%m%d%H%M%S)          # a rebuild of the same key keeps both
    mkdir -p "$d"
    cp "$bit" "$d/$PROJ.bit"
    [ -f "$rpt" ] && cp "$rpt" "$d/timing_summary.rpt"
    grep -o 'Verilog_Define Name="[^"]*" Val="[^"]*"' "$plat/$PROJ.xpr" | sed 's/Verilog_Define Name="//; s/" Val="/=/; s/"$//; s/&apos;/'"'"'/g; s/&quot;/"/g' > "$d/defines.txt" || true
    (cd "$here" && git diff HEAD -- "${diff_paths[@]}" "${excl[@]}") > "$d/dirty.diff" 2>/dev/null || true
    [ -s "$d/dirty.diff" ] || rm -f "$d/dirty.diff"
    wns=$(grep -A2 'WNS(ns)' "$d/timing_summary.rpt" 2>/dev/null | sed -n 3p | awk '{print $1}')
    { echo "key:     $k"
      echo "commit:  $(cd "$here" && git log -1 --format='%H %s')"
      echo "tree:    $here"
      echo "built:   $(date '+%Y-%m-%d %H:%M:%S')"
      echo "wns:     ${wns:-?}"
      echo "met:     $(awk -v w="${wns:-x}" 'BEGIN{print (w ~ /^-?[0-9.]+$/ && w >= 0) ? "yes" : "NO"}')"
    } > "$d/meta.txt"
    echo "bitcache: banked $d (WNS ${wns:-?})"
    ;;
get)
    pre=${1:?commit prefix}; filt=${2:-}
    for d in $(ls -td "$BITCACHE"/"$pre"* 2>/dev/null); do
        case "$d" in *"$filt"*) grep -q '^met: *yes' "$d/meta.txt" 2>/dev/null && { echo "$d/$PROJ.bit"; exit 0; };; esac
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
