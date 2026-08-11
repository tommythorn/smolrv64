#!/bin/bash
# Static lint gate for the RTL (docs/rtl-rules.md E1).
#
#   ./lint.sh              # gate: fail on any load-bearing rule
#   ./lint.sh -v           # also print the advisory classes we are not clean on yet
#
# The load-bearing classes are ERRORS. They are the ones the defect record is made
# of: silent truncation at an array bracket or a port boundary, an FSM case with no
# default, an inferred latch holding stale control, a combinational loop, an output
# nobody drives, a module defined twice. Style classes stay off -- six of the twelve
# suppressions the build used to carry were load-bearing; the rest are noise.
#
# Waivers live in verilator.vlt and must name a file. Never add a global -Wno- here.
set -u
cd "$(dirname "$0")"

srcs=$(. ./rtl-sources.sh; rtl_sources)

# Rules that fail the gate. Only WIDTHEXPAND (87 benign zero-extensions) is still
# advisory. PINMISSING was promoted once the probe-only variants left the source list
# and the one real hit -- the iMMU's t_uncached -- was named-and-empty on purpose.
# Shrink the advisory list, do not grow this one.
ERRS="-Werror-WIDTHTRUNC -Werror-CASEINCOMPLETE -Werror-LATCH -Werror-UNOPTFLAT
      -Werror-UNDRIVEN -Werror-MODDUP -Werror-IMPLICIT -Werror-PINNOTFOUND
      -Werror-BLKANDNBLK -Werror-MULTIDRIVEN -Werror-PINMISSING"

# Style/noise: off by name, so the list is auditable.
OFF="-Wno-TIMESCALEMOD -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM -Wno-DECLFILENAME
     -Wno-ASCRANGE -Wno-UNSIGNED -Wno-VARHIDDEN -Wno-SYNCASYNCNET -Wno-GENUNNAMED
     -Wno-PINCONNECTEMPTY -Wno-PROCASSINIT -Wno-BLKSEQ -Wno-WIDTHEXPAND"

echo "lint: verilator $(verilator --version 2>&1 | head -1)"
verilator --lint-only --timing -sv -Wall $OFF $ERRS ${VDEFS:-} \
   -I. --top-module soc_top \
   $srcs -f ./cvfpu_sources.f ./smolrv64_cvfpu.sv fp_unit.sv \
   ./verilator.vlt > /tmp/smolrv64-lint.log 2>&1
rc=$?

if [ "${1:-}" = "-v" ]; then
   echo "---- advisory (not gating) ----"
   grep -oE '%Warning-[A-Z]+' /tmp/smolrv64-lint.log | sort | uniq -c | sort -rn
fi

if [ $rc -ne 0 ]; then
   echo "---- LINT FAILED ----"
   grep -E '%Error' /tmp/smolrv64-lint.log | head -40
   echo "(full log: /tmp/smolrv64-lint.log)"
   exit 1
fi
echo "lint: clean"
