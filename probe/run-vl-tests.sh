#!/bin/bash
# Fast parallel regression for the sharded-OoO probe, verilated.
#   ./run-vl-tests.sh [class-glob ...]      e.g. rv64ui-p rv64ui-v
# Default: all standard classes. Builds the single tb_vl binary once (verilator
# --binary --timing, warnings on), then runs every matching test in parallel
# (JOBS at a time, default = nproc) via that one binary with +hex/+tohost.
set -u
cd "$(dirname "$0")"

TESTDIR=../tests/riscv-tests/passes
NM=$(command -v riscv64-unknown-elf-nm || command -v riscv64-elf-nm || command -v riscv64-linux-gnu-nm)
CYC=${CYC:-200000}
JOBS=${JOBS:-$(nproc 2>/dev/null || echo 8)}
classes=("$@")
[ ${#classes[@]} -eq 0 ] && classes=(rv64ui-p rv64um-p rv64uc-p rv64ua-p rv64uf-p rv64ud-p rv64mi-p rv64si-p \
                                     rv64ui-v rv64um-v rv64ua-v rv64uc-v)

# ---- build the verilated binary once ----
srcs=$(ls *.v | grep -vE '^tb_|probe|^flopwrap.v$|^rf_alu.v')
echo "building obj_dir_vl/tb_vl ..."
# The core embeds the CVFPU (smolrv64_cvfpu.sv via fp_unit.sv) for the F/D extensions, so
# the FP source list + SystemVerilog + the cvfpu-specific -Wno flags are always needed.
verilator --binary --timing -j 0 -sv -Wall \
   -Wno-fatal -Wno-TIMESCALEMOD -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC \
   -Wno-CASEINCOMPLETE -Wno-LATCH -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM -Wno-DECLFILENAME \
   -Wno-ASCRANGE -Wno-UNSIGNED -Wno-WIDTH -Wno-UNOPTFLAT \
   -I. -I../src --top-module tb --Mdir obj_dir_vl -o tb_vl \
   $srcs tb_vl.v ../src/alu.v -f ../src/cvfpu_sources.f ../src/smolrv64_cvfpu.sv fp_unit.sv \
   > /tmp/vlbuild.log 2>&1
if [ $? -ne 0 ]; then echo "BUILD FAILED:"; grep -E '%Error' /tmp/vlbuild.log; exit 1; fi
BIN=$(pwd)/obj_dir_vl/tb_vl

tmp=$(mktemp -d); trap 'rm -rf "$tmp"' EXIT
res="$tmp/results"; : > "$res"
run_one() {
   local bin_t="$1" base="$2" elf="$3"
   # need the ELF's tohost symbol; skip when the ELF is absent (some tests ship only .bin,
   # e.g. fcvt_w) -- a guessed tohost just yields a spurious fail/timeout.
   local th; th=$("$NM" "$elf" 2>/dev/null | awk '/ tohost$/{print $1}')
   if [ -z "$th" ]; then printf "%-26s SKIP(no-elf)\n" "$base"; return; fi
   od -An -v -tx1 "$bin_t" > "$tmp/$base.hex"
   local out; out=$("$BIN" +hex="$tmp/$base.hex" +tohost="$th" +cycles=$CYC +memlat=${MEMLAT:-0} +cache=${CACHE:-0} 2>&1 | grep -E 'RISCV-TEST')
   case "$out" in
      *PASS*)    printf "%-26s PASS\n"    "$base" ;;
      *TIMEOUT*) printf "%-26s TIMEOUT\n" "$base"; echo "FAIL $base" >> "$res" ;;
      *)         printf "%-26s %s\n" "$base" "${out#*RISCV-TEST }"; echo "FAIL $base" >> "$res" ;;
   esac
}

for cls in "${classes[@]}"; do
   for bin in "$TESTDIR"/${cls}*.bin; do
      [ -e "$bin" ] || continue
      base=$(basename "$bin" .bin)
      run_one "$bin" "$base" "$TESTDIR/$base" &
      while [ "$(jobs -pr | wc -l)" -ge "$JOBS" ]; do wait -n; done
   done
done
wait

nfail=$(grep -c '^FAIL' "$res" 2>/dev/null); nfail=${nfail:-0}
echo "----"
echo "failures: $nfail"
[ "$nfail" -gt 0 ] && grep '^FAIL' "$res"
exit 0
