#!/usr/bin/env bash
# The FPU on a board against tools/fpref.py: every FP arithmetic instruction (the FMA group, the
# conversions, division, square root), both formats, every rounding mode (frm, the instruction's rm
# dynamic), result bits and fflags. fpchk.c is compiled on the board.
#   tools/board/fpchk/fpchk.sh <board-ip> [n]     n FMA-unit vectors, n/4 each for div and sqrt (200000)
set -euo pipefail
B=$1; N=${2:-200000}
here=$(cd "$(dirname "$0")" && pwd); repo=$(cd "$here/../../.." && pwd)
t=$(mktemp -d); trap 'rm -rf "$t"' EXIT
python3 "$repo/tools/fpref.py" vectors fma  "$N"         31 > "$t/fma.vec"
python3 "$repo/tools/fpref.py" vectors div  $((N / 4)) 32 > "$t/div.vec"
python3 "$repo/tools/fpref.py" vectors sqrt $((N / 4)) 33 > "$t/sqrt.vec"
ssh -o BatchMode=yes "$B" 'mkdir -p /tmp/fpchk'
scp -q -o BatchMode=yes "$here/fpchk.c" "$t"/*.vec "$B:/tmp/fpchk/"
ssh -o BatchMode=yes "$B" 'tr -d "\0" < /proc/device-tree/model; echo; cd /tmp/fpchk && gcc -O2 -o fpchk fpchk.c && rc=0 && for o in fma div sqrt; do ./fpchk $o $o.vec | tail -16 || rc=1; done; exit $rc'
