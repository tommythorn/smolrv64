#!/bin/bash
# Linux-boot lockstep: soc_top vs simmerv. Builds tb_cosim_linux (soc_top + caches +
# CLINT/PLIC/UART + behavioral DDR) with -DPROBE_COSIM, resets to OpenSBI (0x8000_0000)
# with a1=DTB, and locksteps every committed instruction against simmerv -- aborts on the
# first divergence (the high-signal output for an overnight run).
#   ./run-cosim-linux.sh            (builds if needed; BUILD=1 forces rebuild)
#   CYC=... ./run-cosim-linux.sh
set -u
cd "$(dirname "$0")"

SIMMERV_DIR=${SIMMERV_DIR:-$HOME/simmerv}
SIMMERV_LIB=$SIMMERV_DIR/target/release/libsimmerv_cosim.a
SIMMERV_INC=$SIMMERV_DIR/cosim
W=../workloads/linux
FW=${FW:-$W/fw_payload.bin}; DTB=${DTB:-$W/dts.dtb}; INITRD=${INITRD:-$W/tiny128.cpio}
CYC=${CYC:-200000000}
BIN=$(pwd)/obj_dir_cosim_linux/tb_cosim_linux

[ -f "$SIMMERV_LIB" ] || (cd "$SIMMERV_DIR" && cargo build --release -p simmerv-cosim) || exit 1

if [ ! -x "$BIN" ] || [ "${BUILD:-0}" = 1 ]; then
   srcs=$(ls *.v | grep -vE '^tb_|probe|^flopwrap.v$|^rf_alu.v')
   echo "building obj_dir_cosim_linux/tb_cosim_linux ..."
   verilator --binary --timing -j 0 -sv -Wall \
      -Wno-fatal -Wno-TIMESCALEMOD -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC \
      -Wno-CASEINCOMPLETE -Wno-LATCH -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM -Wno-DECLFILENAME \
      -Wno-ASCRANGE -Wno-UNSIGNED -Wno-WIDTH -Wno-UNOPTFLAT \
      -DPROBE_COSIM -DCOSIM_MEM_SIZE_LG2=28 ${VDEFS:-} \
      -CFLAGS "-O2 -DCOSIM_MEM_SIZE_LG2=28 -I$SIMMERV_INC" \
      -LDFLAGS "$SIMMERV_LIB -lpthread -ldl -lm" \
      -I. -I../src --top-module tb --Mdir obj_dir_cosim_linux -o tb_cosim_linux \
      $srcs tb_cosim_linux.v ../src/alu.v -f ../src/cvfpu_sources.f ../src/smolrv64_cvfpu.sv \
      fp_unit.sv ../src/smolrv64_plic_arbiter.v probe_cosim.cpp > /tmp/cosimlinuxbuild.log 2>&1
   if [ $? -ne 0 ]; then echo "BUILD FAILED:"; grep -E '%Error' /tmp/cosimlinuxbuild.log | head; exit 1; fi
fi

echo "=== Linux cosim (fw=$FW dtb=$DTB) ==="
"$BIN" +fw="$FW" +dtb="$DTB" +initrd="$INITRD" +a1=82000000 +cycles=$CYC
