#!/bin/bash
# Lockstep the sharded-OoO probe against simmerv. Builds tb_vl.v with
# -DPROBE_COSIM (backend_top emits a probe_retire() DPI stream) linked against
# probe_cosim.cpp + libsimmerv_cosim.a, then runs one riscv-test (or any flat
# image) comparing every committed instruction to the golden model.
#
#   ./run-cosim.sh <test-base>      e.g. rv64ui-p-add   (builds if needed)
#   BUILD=1 ./run-cosim.sh <test>   force a rebuild
#   RESET_PC=80000000 CYC=... ./run-cosim.sh <test>
set -u
cd "$(dirname "$0")"

SIMMERV_DIR=${SIMMERV_DIR:-$HOME/simmerv}
SIMMERV_LIB=$SIMMERV_DIR/target/release/libsimmerv_cosim.a
SIMMERV_INC=$SIMMERV_DIR/cosim
TESTDIR=../tests/riscv-tests/passes
NM=$(command -v riscv64-unknown-elf-nm || command -v riscv64-elf-nm || command -v riscv64-linux-gnu-nm)
CYC=${CYC:-2000000}
RESET_PC=${RESET_PC:-80000000}
BIN=$(pwd)/obj_dir_cosim/tb_cosim

if [ ! -f "$SIMMERV_LIB" ]; then
   echo "building simmerv cosim lib ..."
   (cd "$SIMMERV_DIR" && cargo build --release -p simmerv-cosim) || exit 1
fi

if [ ! -x "$BIN" ] || [ "${BUILD:-0}" = 1 ]; then
   srcs=$(ls *.v | grep -vE '^tb_|probe|^flopwrap.v$|^rf_alu.v')
   echo "building obj_dir_cosim/tb_cosim ..."
   verilator --binary --timing -j 0 -sv -Wall \
      -Wno-fatal -Wno-TIMESCALEMOD -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC \
      -Wno-CASEINCOMPLETE -Wno-LATCH -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM -Wno-DECLFILENAME \
      -Wno-ASCRANGE -Wno-UNSIGNED -Wno-WIDTH -Wno-UNOPTFLAT \
      -DPROBE_COSIM ${VDEFS:-} \
      -CFLAGS "-O2 -I$SIMMERV_INC" -LDFLAGS "$SIMMERV_LIB -lpthread -ldl -lm" \
      -I. --top-module tb --Mdir obj_dir_cosim -o tb_cosim \
      $srcs tb_vl.v ./alu.v -f ./cvfpu_sources.f ./smolrv64_cvfpu.sv fp_unit.sv \
      probe_cosim.cpp > /tmp/cosimbuild.log 2>&1
   if [ $? -ne 0 ]; then echo "BUILD FAILED:"; grep -E '%Error' /tmp/cosimbuild.log | head; exit 1; fi
fi

base=${1:?usage: run-cosim.sh <test-base>}
bin_t="$TESTDIR/$base.bin"
elf="$TESTDIR/$base"
[ -e "$bin_t" ] || { echo "no image $bin_t"; exit 1; }
th=$("$NM" "$elf" 2>/dev/null | awk '/ tohost$/{print $1}')
[ -z "$th" ] && { echo "no tohost symbol in $elf"; exit 1; }
hex=$(mktemp)
od -An -v -tx1 "$bin_t" > "$hex"
echo "=== cosim $base (tohost=$th reset_pc=$RESET_PC) ==="
"$BIN" +hex="$hex" +tohost="$th" +cycles=$CYC +reset_pc=$RESET_PC 2>&1
rm -f "$hex"
