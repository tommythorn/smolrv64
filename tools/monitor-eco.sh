#!/usr/bin/env bash
# Put a new ROM monitor into a met build without a new implementation: its routed checkpoint's
# boot-SRAM block RAMs get the new image's INIT strings (tools/monitor-eco.tcl, the packing in
# tools/lmem-init.py) and a bitstream is written from it. Timing is the build's own; the ECO
# refuses a checkpoint that does not hold exactly the old image.
#
#   tools/monitor-eco.sh <bitcache entry> <old monitor.bin> <new monitor.bin> <out dir>
#
# The old image is the build's (its meta.txt's "monitor:" is the sha1 of its line-hex, checked
# here); the out dir gets the bitstream, routed.dcp, the INIT dumps before and after, and meta.txt.
set -eu
here=$(cd "$(dirname "$0")/.." && pwd)
[ $# -eq 4 ] || { sed -n '2,10p' "$0"; exit 2; }
entry=$(realpath "$1"); old=$(realpath "$2"); new=$(realpath "$3"); out=$4
mkdir -p "$out"; out=$(realpath "$out")
proj=$(sed -n 's/^project: *//p' "$entry/meta.txt")
want=$(sed -n 's/^monitor: *//p' "$entry/meta.txt")
have=$(python3 "$here/src/binline.py" "$old" | sha1sum | cut -c1-12)
[ "$want" = "$have" ] || { echo "monitor-eco: $old is $have, the build's monitor is $want" >&2; exit 1; }
plat=$(dirname "$(ls "$here"/platforms/*/"$proj".srcs -d)")
VIVADO=$(make -s -C "$plat" print-VIVADO); LIB=$(make -s -C "$plat" print-VIVADO_LIB)
LD_LIBRARY_PATH=$LIB "$VIVADO" -mode batch -nojournal -log "$out/vivado.log" \
    -source "$here/tools/monitor-eco.tcl" -tclargs "$entry/routed.dcp" "$old" "$new" "$out" "$proj.bit" < /dev/null
{
    echo "base:    $(basename "$entry")"
    sed -n 's/^\(commit\|wns\|whs\):/base \1:/p' "$entry/meta.txt"
    echo "monitor: $(python3 "$here/src/binline.py" "$new" | sha1sum | cut -c1-12) (was $want)"
    echo "monitor from: $(git -C "$here" log -1 --format='%h %s' -- workloads/monitor) $(git -C "$here" diff --quiet -- workloads/monitor || echo '+ uncommitted changes')"
    echo "eco:     $(date '+%F %T')"
} > "$out/meta.txt"
cat "$out/meta.txt"
