#!/usr/bin/env bash
# The failing-path census of routed checkpoints: tools/timing-census.sh <dcp>... [-o <dir>]
# Each checkpoint's failing endpoints (tools/timing-census.tcl, one Vivado session a checkpoint, run
# one after another: never beside a build) into <dir>/<name>.txt, then the families across them
# (tools/timing-census.py). <name> is the checkpoint's directory name. Run it on every draw that
# misses: the frontend-against-the-rest column says whether a family's cut can move WNS at all.
set -e
here=$(cd "$(dirname "$0")" && pwd)
out=./census; dcps=()
while [ $# -gt 0 ]; do case $1 in -o) out=$2; shift 2;; *) dcps+=("$1"); shift;; esac; done
[ ${#dcps[@]} -gt 0 ] || { echo "usage: tools/timing-census.sh <dcp>... [-o <dir>]"; exit 2; }
mkdir -p "$out"
VIVADO_DIR=${VIVADO_DIR:-/home/tommy/Xilinx/2025.1/Vivado}
VIV="env LD_LIBRARY_PATH=$VIVADO_DIR/lib/lnx64.o/Ubuntu/24 $VIVADO_DIR/bin/vivado -mode batch -nojournal -nolog"
txts=()
for d in "${dcps[@]}"; do
   n=$(basename "$(dirname "$(readlink -f "$d")")")
   (cd "$out" && $VIV -source "$here/timing-census.tcl" -tclargs "$(readlink -f "$d")" "$n.txt" > "$n.log" 2>&1) \
      || { echo "timing-census: Vivado failed on $d (log: $out/$n.log)"; exit 1; }
   txts+=("$out/$n.txt")
done
python3 "$here/timing-census.py" "${txts[@]}"
