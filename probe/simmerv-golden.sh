#!/bin/bash
# Golden-reference trace of a riscv-test from simmerv (the RV64GC ISA model),
# for comparing against the probe backend when a test diverges.
#
#   ./simmerv-golden.sh <test-name> [> golden.trace]
#   e.g. ./simmerv-golden.sh rv64ui-v-ma_data | grep ' 0000000000002a04 '
#
# simmerv runs the test ELF and emits one line per RETIRED instruction:
#   <count> <priv> <pc> <insn> <disasm> <result>
# This is the in-order architectural truth. The probe is OoO/CPR (no per-insn
# retire stream), so compare coarse observables: the committed store stream
# (tb_riscv dmem_wen) and divergent register values at a suspect PC. To find
# WHERE control flow first goes wrong, grep the golden trace for the PC the
# probe traps/branches at and read the correct insn + operands there.
set -u
SIMMERV_DIR=${SIMMERV_DIR:-$HOME/github/simmerv}
TESTDIR=$(cd "$(dirname "$0")/../tests/riscv-tests/passes" && pwd)
test=${1:?usage: simmerv-golden.sh <test-name>}
elf="$TESTDIR/$test"
[ -f "$elf" ] || { echo "no such test: $elf" >&2; exit 1; }
# -t = per-instruction trace, -n = no terminal. Build is cached after the first run.
( cd "$SIMMERV_DIR" && cargo r -r -q -- "$elf" -t -n 2>&1 )
