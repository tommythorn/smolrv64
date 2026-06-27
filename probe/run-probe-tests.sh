#!/bin/bash
# Run riscv-tests against the sharded-OoO probe backend.
#   ./run-probe-tests.sh [class-glob ...]      e.g. rv64ui-p rv64um-p
# Default: rv64ui-p.  Compiles the harness once, then runs each test via vvp
# with +hex (byte image) and +tohost (from the ELF symbol).
set -u
cd "$(dirname "$0")"

# Safety net: cap address space at 25 GiB so a runaway sim (e.g. a zero-delay
# combinational oscillation) aborts on bad_alloc instead of OOM-killing the box.
ulimit -v $((25 * 1024 * 1024)) 2>/dev/null || true

TESTDIR=../tests/riscv-tests/passes
NM=$(command -v riscv64-unknown-elf-nm || command -v riscv64-elf-nm || command -v riscv64-linux-gnu-nm)
CYC=${CYC:-200000}
classes=("$@"); [ ${#classes[@]} -eq 0 ] && classes=(rv64ui-p)

# compile harness once
srcs=$(ls *.v | grep -vE '^tb_|probe|^flopwrap.v$|^rf_alu.v')
iverilog -g2012 -I. -I../src -s tb -o /tmp/probe_riscv.vvp $srcs tb_riscv.v ../src/alu.v ../src/smolrv64_sdpram.v || exit 1

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
      out=$(vvp /tmp/probe_riscv.vvp +hex="$tmp/img.hex" +tohost="$th" +cycles=$CYC 2>&1 \
            | grep -E 'RISCV-TEST')
      case "$out" in
         *PASS*)    printf "%-26s PASS\n"    "$base"; pass=$((pass+1)) ;;
         *TIMEOUT*) printf "%-26s TIMEOUT\n" "$base"; to=$((to+1)) ;;
         *)         printf "%-26s %s\n" "$base" "${out#*RISCV-TEST }"; fail=$((fail+1)) ;;
      esac
   done
done
echo "----"
echo "pass=$pass fail=$fail timeout=$to"
