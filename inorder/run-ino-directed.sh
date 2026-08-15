#!/bin/bash
# Directed tests for behaviour riscv-tests does not reach.
#
# riscv-tests is a conformance suite for things a correct program does; it never
# issues a misaligned atomic, reads an unimplemented CSR, and so on. Each test here
# exists because a real defect hid behind that gap -- so each one should FAIL on the
# core as it was before the corresponding fix. Keep them cheap and self-checking:
# tohost=1 is PASS, anything else is FAIL with test# = tohost>>1.
#
#   ./run-ino-directed.sh              all tests in directed/
#   ./run-ino-directed.sh amomis       just one
#
# Assumes obj_dir_ino/tb_ino is current (./run-ino-vl.sh builds it).
set -u
cd "$(dirname "$0")"

BIN=$(pwd)/obj_dir_ino/tb_ino
CYC=${CYC:-400000}
CC=$(command -v riscv64-unknown-elf-gcc || command -v riscv64-elf-gcc || command -v riscv64-linux-gnu-gcc)
OC=${CC%gcc}objcopy
NM=${CC%gcc}nm

[ -x "$BIN" ] || { echo "ERROR: $BIN missing -- run ./run-ino-vl.sh first" >&2; exit 1; }

tests=("$@")
[ ${#tests[@]} -eq 0 ] && tests=($(cd directed && ls *.S | sed 's/\.S$//'))

tmp=$(mktemp -d); trap 'rm -rf "$tmp"' EXIT
fail=0
for t in "${tests[@]}"; do
    # zicsr is not implied by rv64ima on modern binutils, and every directed test
    # touches CSRs to observe the trap it provokes.
    "$CC" -march=rv64ima_zicsr -mabi=lp64 -mcmodel=medany -nostdlib -nostartfiles \
          -T directed/"$t".ld -o "$tmp/$t" directed/"$t".S || { echo "$t BUILD FAIL"; fail=1; continue; }
    "$OC" -O binary "$tmp/$t" "$tmp/$t.bin"
    th=$("$NM" "$tmp/$t" | awk '/ tohost$/{print $1}'); [ -z "$th" ] && th=80001000
    od -An -v -tx1 "$tmp/$t.bin" > "$tmp/$t.hex"
    out=$("$BIN" +hex="$tmp/$t.hex" +tohost="$th" +cycles=$CYC 2>&1 | grep -E 'RISCV-TEST')
    case "$out" in
        *PASS*) printf "%-20s PASS\n" "$t" ;;
        *)      printf "%-20s %s\n" "$t" "${out#*RISCV-TEST }"; fail=1 ;;
    esac
done
exit $fail
