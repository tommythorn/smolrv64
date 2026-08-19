#!/bin/bash
# Verilated regression for the in-order core, with the REAL CVFPU.
#   ./run-ino-vl.sh [class-glob ...]      e.g. rv64uf-p rv64ud-p
# Default: every standard class, F/D included.
#
# This is the flow that covers F/D: fpnew uses SystemVerilog concurrent assertions
# iverilog cannot parse, so ./run-ino-tests.sh builds against fp_unit_stub.sv and is
# the fast INTEGER regression. Builds one binary, then runs tests JOBS at a time.
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
            ../src/decode_fp.v ../src/predictor.v ../src/exec_alu.v \
            ../src/branch_unit.v ../src/mul3.v ../src/divider.v \
            ../src/csr_file.v ../src/mmu.v ../src/fp_unit.sv"

echo "building obj_dir_ino/tb_ino ..."
verilator --binary --timing -j 0 -sv -Wall \
   -Wno-fatal -Wno-TIMESCALEMOD -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC \
   -Wno-CASEINCOMPLETE -Wno-LATCH -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM -Wno-DECLFILENAME \
   -Wno-ASCRANGE -Wno-UNSIGNED -Wno-WIDTH -Wno-UNOPTFLAT ${VDEFS:-} \
   -I. -I../probe -I../src --top-module tb --Mdir obj_dir_ino -o tb_ino \
   ino_core.v ino_frontend.v ino_predictor.v ino_exec.v ino_lsu.v ino_regfile.v \
   $PROBE_SRCS ../src/alu.v -f ../src/cvfpu_sources.f ../src/smolrv64_cvfpu.sv \
   tb_ino_riscv.v > /tmp/inovlbuild.log 2>&1
if [ $? -ne 0 ]; then echo "BUILD FAILED:"; grep -E '%Error' /tmp/inovlbuild.log | head -20; exit 1; fi
BIN=$(pwd)/obj_dir_ino/tb_ino

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
