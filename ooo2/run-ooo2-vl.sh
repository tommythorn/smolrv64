#!/bin/bash
# Verilated regression for the core, with the REAL CVFPU.
#   ./run-ooo2-vl.sh [class-glob ...]      e.g. rv64uf-p rv64ud-p
# Default: every standard class, F/D included.
#
# The one riscv-tests flow: Verilator with the real CVFPU. (The iverilog flow with the FP
# stub was dropped in the 2026-09 release: iverilog cannot elaborate the frontend.) Builds
# one binary, then runs tests JOBS at a time.
set -u
cd "$(dirname "$0")"

TESTDIR=../tests/riscv-tests/passes
NM=$(command -v riscv64-unknown-elf-nm || command -v riscv64-elf-nm || command -v riscv64-linux-gnu-nm)
CYC=${CYC:-400000}
JOBS=${JOBS:-$(sysctl -n hw.ncpu 2>/dev/null || nproc 2>/dev/null || echo 8)}
classes=("$@")
[ ${#classes[@]} -eq 0 ] && classes=(rv64ui-p rv64um-p rv64uc-p rv64ua-p rv64uf-p rv64ud-p \
                                     rv64mi-p rv64si-p rv64ssvnapot-p \
                                     rv64ui-v rv64um-v rv64ua-v rv64uc-v rv64uf-v rv64ud-v)

PROBE_SRCS="../src/fetch.v ../src/aligner.v ../src/rvc_expand.v \
            ../src/decode_slot.v ../src/decode_operands.v ../src/decode_exec.v \
            ../src/decode_fp.v ../src/exec_alu.v \
            ../src/branch_unit.v ../src/mul3.v ../src/divider.v \
            ../src/csr_file.v ../src/mmu.v ../src/fp_unit.sv"

echo "building obj_dir_ooo2/tb_ooo2 ..."
verilator --binary --timing -j 0 -sv -Wall \
   -Wno-fatal -Wno-TIMESCALEMOD -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC \
   -Wno-CASEINCOMPLETE -Wno-LATCH -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM -Wno-DECLFILENAME \
   -Wno-ASCRANGE -Wno-UNSIGNED -Wno-WIDTH -Wno-UNOPTFLAT ${VDEFS:-} \
   -I. -I../src --top-module tb --Mdir obj_dir_ooo2 -o tb_ooo2 \
   ooo2_core.v ooo2_pending.v ooo2_frontend.v ooo2_predictor.v ooo2_exec.v ooo2_lsu.v rv_regfile.v \
   $PROBE_SRCS ../src/alu.v -f ../src/cvfpu_sources.f \
   tb_ooo2_riscv.v > /tmp/ooo2vlbuild.log 2>&1
if [ $? -ne 0 ]; then echo "BUILD FAILED:"; grep -E '%Error' /tmp/ooo2vlbuild.log | head -20; exit 1; fi
BIN=$(pwd)/obj_dir_ooo2/tb_ooo2

tmp=$(mktemp -d); trap 'rm -rf "$tmp"' EXIT
res="$tmp/results"; : > "$res"
run_one() {
   local bin=$1 base elf th
   base=$(basename "$bin" .bin); elf="$TESTDIR/$base"
   th=$("$NM" "$elf" 2>/dev/null | awk '/ tohost$/{print $1}'); [ -z "$th" ] && th=80001000
   od -An -v -tx1 "$bin" > "$tmp/$base.hex"
   local out
   out=$("$BIN" +hex="$tmp/$base.hex" +tohost="$th" +cycles=$CYC 2>&1 | grep -E 'RISCV-TEST')
   case "$out" in
      *PASS*)    printf "%-26s PASS\n"    "$base" ;;
      *TIMEOUT*) printf "%-26s TIMEOUT\n" "$base" ;;
      *)         printf "%-26s %s\n" "$base" "${out#*RISCV-TEST }" ;;
   esac
}
# Batch of JOBS at a time. `wait -n` would be the natural throttle but macOS ships
# bash 3.2, which does not have it -- and the failure is SILENT (every iteration
# errors out and the loop spawns everything at once), so batch instead.
n=0
for cls in "${classes[@]}"; do
   for bin in "$TESTDIR"/${cls}*.bin; do
      [ -e "$bin" ] || continue
      run_one "$bin" >> "$res" &
      n=$((n+1))
      if [ $((n % JOBS)) -eq 0 ]; then wait; fi
   done
done
wait
sort "$res"
echo "---- pass=$(grep -c ' PASS$' "$res") fail=$(grep -vc ' PASS$' "$res")"
