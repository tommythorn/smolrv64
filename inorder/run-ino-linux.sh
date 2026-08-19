#!/bin/bash
# Boot Linux on the in-order SoC under Verilator.
#
# Defaults = the tiny128 initrd workload the OoO core uses for its Linux cosim:
# workloads/ubuntu/fw_payload.bin (OpenSBI + the Ubuntu 6.x kernel -- the blessed
# one; the old workloads/linux 5.4 payload dies with "FATAL: kernel too old"
# against the tiny128 userspace) + tiny128.cpio + tiny128-cosim.dtb, which is
# virtio-free, so no block device is needed.
#
#   ./run-ino-linux.sh              boot with the defaults
#   CYC=0 ./run-ino-linux.sh        unbounded (wrap in `timeout`!)
#   BUILD=1 ./run-ino-linux.sh      force a rebuild
#   FW=... DTB=... INITRD=... OFF_DTB=... OFF_INITRD=... A1=... MEM_LG2=...
#
# FAST ITERATION: tiny128.cpio is a ZSTD stream (3.3 MB -> 10,305,536 bytes), and
# the kernel decompresses it in software -- ~1B extra cycles on this core, during
# which the console is silent because wait_for_initramfs() blocks right before
# "Run /init". Feed the decompressed cpio to skip all of it:
#     zstd -d --stdout ../workloads/tiny128/tiny128.cpio > /tmp/tiny128-raw.cpio
#     INITRD=/tmp/tiny128-raw.cpio ./run-ino-linux.sh
# That is also the layout the DTB describes: tiny128-cosim.dts declares
# linux,initrd-start/end = 0x9f52c000..0x9ff00000 = exactly the DECOMPRESSED size,
# flush-packed against the DTB base.
set -u
cd "$(dirname "$0")"

W=../workloads/tiny128
FW=${FW:-../workloads/ubuntu/fw_payload.bin}
DTB=${DTB:-$W/tiny128-cosim.dtb}
INITRD=${INITRD:-$W/tiny128.cpio}
OFF_DTB=${OFF_DTB:-1ff00000}; OFF_INITRD=${OFF_INITRD:-1f52c000}; A1=${A1:-9ff00000}
MEM_LG2=${MEM_LG2:-29}
CYC=${CYC:-200000000}
BIN=$(pwd)/obj_dir_ino_linux/tb_ino_linux

PROBE_SRCS="../src/fetch.v ../src/aligner.v ../src/rvc_expand.v \
            ../src/decode_slot.v ../src/decode_operands.v ../src/decode_exec.v \
            ../src/decode_fp.v ../src/predictor.v ../src/exec_alu.v \
            ../src/branch_unit.v ../src/mul3.v ../src/divider.v \
            ../src/csr_file.v ../src/mmu.v ../src/fp_unit.sv \
            ino_cache.v ino_l2_arbiter.v ../src/clint.v ../src/plic.v \
            ../src/ddr_hpm.v"

if [ ! -x "$BIN" ] || [ "${BUILD:-0}" = 1 ]; then
   echo "building obj_dir_ino_linux/tb_ino_linux (MEM_LG2=$MEM_LG2) ..."
   verilator --binary --timing -j 0 -sv -Wall \
      -Wno-fatal -Wno-TIMESCALEMOD -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC \
      -Wno-CASEINCOMPLETE -Wno-LATCH -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM -Wno-DECLFILENAME \
      -Wno-ASCRANGE -Wno-UNSIGNED -Wno-WIDTH -Wno-UNOPTFLAT \
      -DINO_MEM_SIZE_LG2=$MEM_LG2 ${VDEFS:-} \
      -I. -I../probe -I../src --top-module tb --Mdir obj_dir_ino_linux -o tb_ino_linux \
      ino_soc_top.v ino_core.v ino_frontend.v ino_predictor.v ino_exec.v ino_lsu.v ino_regfile.v \
      $PROBE_SRCS ../src/alu.v ../src/smolrv64_sdpram.v ../src/smolrv64_plic_arbiter.v \
      -f ../src/cvfpu_sources.f ../src/smolrv64_cvfpu.sv \
      tb_ino_linux.v > /tmp/inolinuxbuild.log 2>&1
   if [ $? -ne 0 ]; then echo "BUILD FAILED:"; grep -E '%Error' /tmp/inolinuxbuild.log | head -20; exit 1; fi
fi

echo "=== booting: fw=$FW dtb=$DTB initrd=${INITRD:-none} a1=$A1 ==="
exec "$BIN" +fw="$FW" +dtb="$DTB" ${INITRD:+ +initrd="$INITRD"} \
     +dtb_off=$OFF_DTB +initrd_off=$OFF_INITRD +a1=$A1 +cycles=$CYC
