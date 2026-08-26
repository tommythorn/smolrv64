#!/bin/bash
# Lockstep the in-order core against simmerv. Builds tb_ooo2_riscv.v with
# -DINO_COSIM (ooo2_core emits a probe_retire() DPI stream) linked against
# ../src/probe_cosim.cpp + libsimmerv_cosim.a, then runs one riscv-test (or any
# flat image), comparing every retired instruction to the golden model.
#
#   ./run-ooo2-cosim.sh <test-base>       e.g. rv64ui-p-add
#   ./run-ooo2-cosim.sh -a [class ...]    sweep whole classes, report divergences
#   BUILD=1 ./run-ooo2-cosim.sh <test>    force a rebuild
#   RESET_PC=80000000 CYC=... ./run-ooo2-cosim.sh <test>
#
# The C++ side is probe/probe_cosim.cpp UNMODIFIED -- the DPI contract is the same
# one the OoO core emits, and it is the RTL side that got simpler (see ooo2_core.v's
# OOO2_COSIM block: no reorder FIFO, no value-ready tracking, no squash truncation).
set -u
cd "$(dirname "$0")"

SIMMERV_DIR=${SIMMERV_DIR:-$HOME/simmerv}
SIMMERV_LIB=$SIMMERV_DIR/target/release/libsimmerv_cosim.a
SIMMERV_INC=$SIMMERV_DIR/cosim
TESTDIR=../tests/riscv-tests/passes
NM=$(command -v riscv64-unknown-elf-nm || command -v riscv64-elf-nm || command -v riscv64-linux-gnu-nm)
CYC=${CYC:-2000000}
RESET_PC=${RESET_PC:-80000000}
BIN=$(pwd)/obj_dir_ooo2_cosim/tb_ooo2_cosim
# libsimmerv_cosim.a carries a vmnet shim (simmerv's virtio-net backend), whose
# symbols live in the vmnet framework on macOS. Harmless for cosim, but the link
# fails without it.
EXTRA_LD=""
[ "$(uname -s)" = "Darwin" ] && EXTRA_LD="-framework vmnet"

if [ ! -f "$SIMMERV_LIB" ]; then
   echo "building simmerv cosim lib ..."
   (cd "$SIMMERV_DIR" && cargo build --release -p simmerv-cosim) || exit 1
fi

PROBE_SRCS="../src/fetch.v ../src/aligner.v ../src/rvc_expand.v \
            ../src/decode_slot.v ../src/decode_operands.v ../src/decode_exec.v \
            ../src/decode_fp.v ../src/predictor.v ../src/exec_alu.v \
            ../src/branch_unit.v ../src/mul3.v ../src/divider.v \
            ../src/csr_file.v ../src/mmu.v ../src/fp_unit.sv"

if [ ! -x "$BIN" ] || [ "${BUILD:-0}" = 1 ]; then
   echo "building obj_dir_ooo2_cosim/tb_ooo2_cosim ..."
   verilator --binary --timing -j 0 -sv -Wall \
      -Wno-fatal -Wno-TIMESCALEMOD -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC \
      -Wno-CASEINCOMPLETE -Wno-LATCH -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM -Wno-DECLFILENAME \
      -Wno-ASCRANGE -Wno-UNSIGNED -Wno-WIDTH -Wno-UNOPTFLAT \
      -DINO_COSIM ${VDEFS:-} \
      -CFLAGS "-O2 -I$SIMMERV_INC" -LDFLAGS "$SIMMERV_LIB -lpthread -ldl -lm $EXTRA_LD" \
      -I. -I../probe -I../src --top-module tb --Mdir obj_dir_ooo2_cosim -o tb_ooo2_cosim \
      ooo2_core.v ooo2_pending.v ooo2_frontend.v ooo2_predictor.v ooo2_exec.v ooo2_lsu.v rv_regfile.v \
      $PROBE_SRCS ../src/alu.v -f ../src/cvfpu_sources.f ../src/smolrv64_cvfpu.sv \
      tb_ooo2_riscv.v ../src/probe_cosim.cpp > /tmp/ooo2cosimbuild.log 2>&1
   if [ $? -ne 0 ]; then echo "BUILD FAILED:"; grep -E '%Error' /tmp/ooo2cosimbuild.log | head -20; exit 1; fi
fi

run_one() {   # $1 = test base; echoes one status line, returns 1 on divergence
   local base=$1 bin_t elf th hex out
   bin_t="$TESTDIR/$base.bin"; elf="$TESTDIR/$base"
   [ -e "$bin_t" ] || { printf "%-26s NO-IMAGE\n" "$base"; return 1; }
   th=$("$NM" "$elf" 2>/dev/null | awk '/ tohost$/{print $1}'); [ -z "$th" ] && th=80001000
   hex=$(mktemp); od -An -v -tx1 "$bin_t" > "$hex"
   out=$("$BIN" +hex="$hex" +tohost="$th" +cycles=$CYC +reset_pc=$RESET_PC 2>&1)
   rm -f "$hex"
   if echo "$out" | grep -q 'MISMATCH'; then
      printf "%-26s DIVERGE\n" "$base"
      echo "$out" | grep -A40 'MISMATCH' | head -50
      return 1
   fi
   if echo "$out" | grep -q 'RISCV-TEST PASS'; then
      # no divergence reported by probe_cosim.cpp AND the test itself passed
      printf "%-26s OK  (%s retires lockstepped)\n" "$base" \
             "$(echo "$out" | sed -n 's/.*RISCV-TEST PASS retires=\([0-9]*\).*/\1/p')"
      return 0
   fi
   printf "%-26s %s\n" "$base" "$(echo "$out" | grep -E 'RISCV-TEST' | head -1)"
   return 1
}

if [ "${1:-}" = "-a" ]; then
   shift
   classes=("$@"); [ ${#classes[@]} -eq 0 ] && classes=(rv64ui-p rv64um-p rv64uc-p rv64ua-p \
                                                       rv64uf-p rv64ud-p rv64mi-p rv64si-p)
   ok=0; bad=0
   for cls in "${classes[@]}"; do
      for b in "$TESTDIR"/${cls}*.bin; do
         [ -e "$b" ] || continue
         if run_one "$(basename "$b" .bin)"; then ok=$((ok+1)); else bad=$((bad+1)); fi
      done
   done
   echo "---- ok=$ok diverged/failed=$bad"
   [ "$bad" -eq 0 ]
else
   base=${1:?usage: run-ooo2-cosim.sh <test-base> | -a [class ...]}
   echo "=== cosim $base (reset_pc=$RESET_PC) ==="
   run_one "$base"
fi
