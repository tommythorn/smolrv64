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

. ./rtl-sources.sh

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

fail=0
# One invocation, two tops. The in-order core ships to the FPGA alongside the OoO core but
# was outside this gate entirely -- rv_soc_top.v, ooo2_lsu.v, rv_cache.v and the rest were
# never width- or latch-checked, which is exactly the blind spot docs/rtl-rules.md exists to
# close. Same ERRS/OFF for both: a rule that is load-bearing for one core is load-bearing
# for the other.
lint_top() {                      # <label> <top-module> <sources...>
   local label=$1 top=$2; shift 2
   local log=/tmp/smolrv64-lint-$label.log
   verilator --lint-only --timing -sv -Wall $OFF $ERRS ${VDEFS:-} \
      -I. -I../ooo2 --top-module "$top" \
      "$@" -f ./cvfpu_sources.f ./smolrv64_cvfpu.sv fp_unit.sv \
      ./verilator.vlt > "$log" 2>&1 || {
         echo "---- LINT FAILED ($label: top=$top) ----"
         grep -E '%Error' "$log" | head -40
         echo "(full log: $log)"
         fail=1
      }
   if [ "${VERBOSE:-0}" = 1 ]; then
      echo "---- advisory, not gating ($label) ----"
      grep -oE '%Warning-[A-Z]+' "$log" | sort | uniq -c | sort -rn
   fi
}

[ "${1:-}" = "-v" ] && VERBOSE=1
lint_top ooo soc_top     $(rtl_sources)
lint_top ooo2 rv_soc_top $(ooo2_sources)

# docs/smolrv64-perf-events.json is generated from csr_file.v's event map. It is checked
# HERE because a generated file nothing verifies is a file that drifts: this one had drifted
# into describing a retired core, naming 0x0300..0x0304 as TLB events while the RTL counts
# the in-order stall attribution there, so anyone resolving a perf event by name got the
# wrong counter.
../tools/gen-perf-events.py --check || fail=1

# The DTB's timebase-frequency is what Linux uses for EVERY deadline, and it is a pure
# function of PROBE_CLK_DIV8 -- but SCALE_DIV is an integer divide, so it is NOT 501253 at
# every clock. It said 501253 from the 66.67 MHz era until 2026-08-19, so every FPGA run
# from the 111 MHz milestone on (the GB5 run included) ran 0.302% fast. The DTS comment
# said "MUST track SCALE_DIV" the whole time; nothing checked it. Now something does.
# DIV8=48 (166.67 MHz) is the shipping clock; pass a different one when that changes. It was
# 72 here while the DTBs had already been regenerated for 48, so the gate was red against a
# clock nothing builds -- and 42508cf0 had meanwhile dropped PROBE_CLK_DIV8 from the Vivado
# project entirely, so the bitstreams were being built at the RTL default 120 (66.67 MHz).
../tools/check-dts-timebase.py 48 ../workloads/ubuntu/ubuntu-nfs.dts \
                                 ../workloads/gb5/gb5-fpga.dts || fail=1

# Rule F4: no function reads an array -- Vivado keeps one read port per such function and
# folds the other call sites to 0 (rename port A wrote p0 on four bitstreams, 2026-09-06).
../tools/check-func-ram-reads.py ../ooo2/*.v ../src/*.v || fail=1

[ $fail -ne 0 ] && exit 1
echo "lint: clean"
