#!/bin/bash
# Linux-boot lockstep: the in-order SoC vs simmerv. Builds tb_ino_linux.v with
# -DINO_COSIM (ino_core emits its probe_retire() stream) linked against
# ../src/probe_cosim.cpp + libsimmerv_cosim.a, resets to OpenSBI (0x8000_0000)
# with a1=DTB, and locksteps every retired instruction -- aborting on the first
# divergence with a 320-deep DUT/REF history ring.
#
# This is the tool for the /init stall (docs/inorder-plan.md): it names the first
# architecturally wrong instruction instead of inferring from a spin address.
#
#   ./run-ino-cosim-linux.sh              default tiny128 initrd workload
#   CYC=0 ./run-ino-cosim-linux.sh        unbounded (wrap in `timeout`)
#   BUILD=1 ./run-ino-cosim-linux.sh      force a rebuild
#   FW=... DTB=... INITRD=... OFF_DTB=... OFF_INITRD=... A1=... MEM_LG2=...
#
# MEM_LG2 sizes THREE things that must agree: the RTL DDR array (INO_MEM_SIZE_LG2),
# the C-side bound + simmerv's memory (COSIM_MEM_SIZE_LG2), and ino_core's valid-DRAM
# window for the MMU's unbacked-PA access-fault check. One knob drives all three.
set -u
cd "$(dirname "$0")"

SIMMERV_DIR=${SIMMERV_DIR:-$HOME/simmerv}
SIMMERV_LIB=$SIMMERV_DIR/target/release/libsimmerv_cosim.a
SIMMERV_INC=$SIMMERV_DIR/cosim
EXTRA_LD=""
[ "$(uname -s)" = "Darwin" ] && EXTRA_LD="-framework vmnet"   # simmerv's vmnet shim

W=../workloads/tiny128
FW=${FW:-../workloads/ubuntu/fw_payload.bin}
DTB=${DTB:-$W/tiny128-cosim.dtb}
INITRD=${INITRD:-$W/tiny128.cpio}
OFF_DTB=${OFF_DTB:-1ff00000}; OFF_INITRD=${OFF_INITRD:-1f52c000}; A1=${A1:-9ff00000}
MEM_LG2=${MEM_LG2:-29}
CYC=${CYC:-0}
BIN=$(pwd)/obj_dir_ino_clinux/tb_ino_clinux

if [ ! -f "$SIMMERV_LIB" ]; then
   echo "building simmerv cosim lib ..."
   (cd "$SIMMERV_DIR" && cargo build --release -p simmerv-cosim) || exit 1
fi

PROBE_SRCS="../src/fetch.v ../src/aligner.v ../src/rvc_expand.v \
            ../src/decode_slot.v ../src/decode_operands.v ../src/decode_exec.v \
            ../src/decode_fp.v ../src/predictor.v ../src/exec_alu.v \
            ../src/branch_unit.v ../src/mul3.v ../src/divider.v \
            ../src/csr_file.v ../src/mmu.v ../src/fp_unit.sv \
            ino_cache.v ino_l2_arbiter.v ../src/clint.v ../src/plic.v \
            ../src/ddr_hpm.v"

if [ ! -x "$BIN" ] || [ "${BUILD:-0}" = 1 ]; then
   echo "building obj_dir_ino_clinux/tb_ino_clinux (MEM_LG2=$MEM_LG2) ..."
   verilator --binary --timing -j 0 -sv -Wall \
      -Wno-fatal -Wno-TIMESCALEMOD -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC \
      -Wno-CASEINCOMPLETE -Wno-LATCH -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM -Wno-DECLFILENAME \
      -Wno-ASCRANGE -Wno-UNSIGNED -Wno-WIDTH -Wno-UNOPTFLAT \
      -DINO_COSIM -DINO_MEM_SIZE_LG2=$MEM_LG2 -DCOSIM_MEM_SIZE_LG2=$MEM_LG2 ${VDEFS:-} \
      -CFLAGS "-O2 -I$SIMMERV_INC -DCOSIM_MEM_SIZE_LG2=$MEM_LG2" \
      -LDFLAGS "$SIMMERV_LIB -lpthread -ldl -lm $EXTRA_LD" \
      -I. -I../probe -I../src --top-module tb --Mdir obj_dir_ino_clinux -o tb_ino_clinux \
      ino_soc_top.v ino_core.v ino_frontend.v ino_predictor.v ino_exec.v ino_lsu.v ino_regfile.v \
      $PROBE_SRCS ../src/alu.v ../src/smolrv64_sdpram.v ../src/smolrv64_plic_arbiter.v \
      -f ../src/cvfpu_sources.f ../src/smolrv64_cvfpu.sv \
      tb_ino_linux.v ../src/probe_cosim.cpp > /tmp/inoclinuxbuild.log 2>&1
   if [ $? -ne 0 ]; then echo "BUILD FAILED:"; grep -E '%Error' /tmp/inoclinuxbuild.log | head -20; exit 1; fi
fi

echo "=== cosim-linux: fw=$FW dtb=$DTB initrd=${INITRD:-none} a1=$A1 mem=2^$MEM_LG2 ==="
exec "$BIN" +fw="$FW" +dtb="$DTB" ${INITRD:+ +initrd="$INITRD"} \
     +dtb_off=$OFF_DTB +initrd_off=$OFF_INITRD +a1=$A1 +cycles=$CYC
