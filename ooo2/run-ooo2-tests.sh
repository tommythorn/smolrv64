#!/bin/bash
# Run riscv-tests against the in-order core.
#   ./run-ooo2-tests.sh [class-glob ...]      e.g. rv64ui-p rv64um-p
# Default: rv64ui-p.  Compiles the harness once, then runs each test via vvp
# with +hex (byte image) and +tohost (from the ELF symbol).
set -u
cd "$(dirname "$0")"

# Safety net: cap address space so a runaway sim aborts on bad_alloc instead of
# OOM-killing the box.
ulimit -v $((25 * 1024 * 1024)) 2>/dev/null || true

TESTDIR=../tests/riscv-tests/passes
NM=$(command -v riscv64-unknown-elf-nm || command -v riscv64-elf-nm || command -v riscv64-linux-gnu-nm)
CYC=${CYC:-200000}
classes=("$@"); [ ${#classes[@]} -eq 0 ] && classes=(rv64ui-p)

# Shared modules compile straight out of ../probe -- ooo2/ never edits them.
#
# fp_unit_stub.sv, NOT the real CVFPU: fpnew uses SystemVerilog concurrent assertions
# iverilog cannot parse. This flow is the fast INTEGER regression; run the F/D classes
# through ./run-ooo2-vl.sh (verilator + the real CVFPU) instead.
PROBE_SRCS="../src/fetch.v ../src/aligner.v ../src/rvc_expand.v \
            ../src/decode_slot.v ../src/decode_operands.v ../src/decode_exec.v \
            ../src/decode_fp.v ../src/predictor.v ../src/exec_alu.v \
            ../src/branch_unit.v ../src/mul3.v ../src/divider.v \
            ../src/csr_file.v ../src/mmu.v ../src/fp_unit_stub.sv"

iverilog -g2012 -I. -I../probe -I../src -s tb -o /tmp/ooo2_riscv.vvp \
   ooo2_core.v ooo2_frontend.v ooo2_predictor.v ooo2_exec.v ooo2_lsu.v rv_regfile.v \
   $PROBE_SRCS ../src/alu.v tb_ooo2_riscv.v || exit 1

tmp=$(mktemp -d); trap 'rm -rf "$tmp"' EXIT
pass=0; fail=0; to=0
for cls in "${classes[@]}"; do
   for bin in "$TESTDIR"/${cls}*.bin; do
      [ -e "$bin" ] || continue
      base=$(basename "$bin" .bin)
      elf="$TESTDIR/$base"
      th=$("$NM" "$elf" 2>/dev/null | awk '/ tohost$/{print $1}')
      [ -z "$th" ] && th=80001000
      od -An -v -tx1 "$bin" > "$tmp/img.hex"
      out=$(vvp /tmp/ooo2_riscv.vvp +hex="$tmp/img.hex" +tohost="$th" +cycles=$CYC 2>&1 \
            | grep -E 'RISCV-TEST')
      case "$out" in
         *PASS*)    printf "%-26s PASS\n"    "$base"; pass=$((pass+1)) ;;
         *TIMEOUT*) printf "%-26s TIMEOUT\n" "$base"; to=$((to+1)) ;;
         *)         printf "%-26s %s\n" "$base" "${out#*RISCV-TEST }"; fail=$((fail+1)) ;;
      esac
   done
done
echo "---- pass=$pass fail=$fail timeout=$to"
