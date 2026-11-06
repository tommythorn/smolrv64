#!/bin/bash
# Boot Linux on the SoC (rv_soc_top) under Verilator.
#
# Defaults = the tiny128 initrd workload the Linux cosim uses:
# workloads/ubuntu/fw_payload.bin (OpenSBI + the Ubuntu 6.x kernel -- the blessed
# one; the old workloads/linux 5.4 payload dies with "FATAL: kernel too old"
# against the tiny128 userspace) + tiny128.cpio + tiny128-cosim.dtb, which is
# virtio-free, so no block device is needed.
#
#   ./run-linux.sh              boot with the defaults
#   CYC=0 ./run-linux.sh        unbounded (wrap in `timeout`!)
#   BUILD=1 ./run-linux.sh      force a rebuild
#   FW=... DTB=... INITRD=... OFF_DTB=... OFF_INITRD=... A1=... MEM_LG2=...
#   CKPT=<dir>                  restore an architectural checkpoint instead of booting:
#                               <dir>/mem.zst into DDR, <dir>/stub.hex into the boot SRAM,
#                               a build that resets there (docs/PLAN-2026-09-25-arch-checkpoints.md);
#                               +measure_from=<retires> +max_retires=<retires> bound the window
#
# FAST ITERATION: tiny128.cpio is a ZSTD stream (3.3 MB -> 10,305,536 bytes), and
# the kernel decompresses it in software -- ~1B extra cycles on this core, during
# which the console is silent because wait_for_initramfs() blocks right before
# "Run /init". Feed the decompressed cpio to skip all of it:
#     zstd -d --stdout ../workloads/tiny128/tiny128.cpio > /tmp/tiny128-raw.cpio
#     INITRD=/tmp/tiny128-raw.cpio ./run-linux.sh
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
[ -n "${CKPT:-}" ] && VDEFS="${VDEFS:-} -DTB_RESET_PC=64'h7000_0000"
# One build directory per config, as the cosim runner keeps them: the default config keeps the
# plain name, any other MEM_LG2/VDEFS (a checkpoint build is one) gets obj_dir_smolrv64_linux.<hash>,
# so a sweep's configs run side by side. Runs of one config share its binary under a shared lock; a
# rebuild takes it exclusively, waiting for the runs in flight (a relinked binary kills them on macOS).
# The check and the build hold a second lock, one runner at a time, so runs started together on a
# stale binary build it once and then all run (a runner queued for the exclusive lock waits out
# every run that holds the shared one).
OBJ=obj_dir_smolrv64_linux
if [ "$MEM_LG2" != 29 ] || [ -n "${VDEFS:-}" ]; then
   OBJ=obj_dir_smolrv64_linux.$(printf '%s|%s' "$MEM_LG2" "${VDEFS:-}" | sha1sum | cut -c1-8)
fi
BIN=$(pwd)/$OBJ/tb_smolrv64_linux
mkdir -p "$OBJ"; exec 8>"$OBJ.build.flock" 9>"$OBJ.flock"; flock -x 8
ZSTD_LD=""; ZSTD_CF=""
[ "$(uname -s)" = "Darwin" ] && { ZSTD_LD="-L$(brew --prefix)/lib"; ZSTD_CF="-I$(brew --prefix)/include"; }

PROBE_SRCS="../src/fetch.v ../src/aligner.v ../src/rvc_expand.v \
            ../src/decode_slot.v ../src/decode_operands.v ../src/decode_exec.v \
            ../src/decode_fp.v ../src/exec_alu.v \
            ../src/branch_unit.v ../src/mul3.v ../src/divider.v \
            ../src/csr_file.v ../src/mmu.v ../src/fp_unit.sv \
            rv_dcache.v rv_icache.v rv_errlog.v rv_mem_arbiter.v rv_l2.v smolrv64_uram_be.v ../src/clint.v ../src/plic.v \
            ../src/ddr_hpm.v"

# THE MODEL IS REBUILT WHEN THE SOURCES CHANGE (rule G5), as the cosim runner does: a verdict
# from a binary that does not contain the change is not a verdict, and this runner used to
# reuse whatever obj_dir_smolrv64_linux held whenever BUILD was unset.
STAMP=$OBJ/.config-stamp
srchash=$(cat *.v *.vh ../src/*.v ../src/*.sv ../src/*.vh ../src/ckpt_dpi.cpp $(grep -v '^+\|^$' ../src/cvfpu_sources.f) 2>/dev/null | sha1sum | cut -c1-16)
want="MEM_LG2=$MEM_LG2 VDEFS=${VDEFS:-} SRC=$srchash"
stale() { [ ! -x "$BIN" ] || [ "$(cat $STAMP 2>/dev/null)" != "$want" ]; }
if stale || [ "${BUILD:-0}" = 1 ]; then
   flock -x 9      # the runs in flight finish first
   echo "building $OBJ/tb_smolrv64_linux (MEM_LG2=$MEM_LG2 VDEFS=${VDEFS:-<none>}) ..."
   verilator --binary --timing -j 0 -sv -Wall \
      -Wno-fatal -Wno-TIMESCALEMOD -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC \
      -Wno-CASEINCOMPLETE -Wno-LATCH -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM -Wno-DECLFILENAME \
      -Wno-ASCRANGE -Wno-UNSIGNED -Wno-WIDTH -Wno-UNOPTFLAT \
      -DSMOLRV64_MEM_SIZE_LG2=$MEM_LG2 ${VDEFS:-} \
      -LDFLAGS "-lzstd $ZSTD_LD" -CFLAGS "$ZSTD_CF" \
      -I. -I../src --top-module tb --Mdir $OBJ -o tb_smolrv64_linux \
      rv_soc_top.v smolrv64_core.v smolrv64_pending.v smolrv64_bankq.v smolrv64_x2.v smolrv64_mwram.v smolrv64_frontend.v smolrv64_fring.v smolrv64_predictor.v smolrv64_exec.v smolrv64_lsu.v \
      $PROBE_SRCS ../src/alu.v ../src/smolrv64_sdpram.v ../src/smolrv64_plic_arbiter.v \
      -f ../src/cvfpu_sources.f \
      ../src/virtio_blk.v ../src/virtio_mmio.v ../src/sd_spi_host.v ../src/dma_master.v ../src/sd_dpi.cpp \
      ../src/ckpt_dpi.cpp tb_smolrv64_linux.v > $OBJ/build.log 2>&1
   if [ $? -ne 0 ]; then echo "BUILD FAILED:"; grep -E '%Error' $OBJ/build.log | head -20; exit 1; fi
   echo "$want" > "$STAMP"
fi
flock -s 9      # shared for the run: the exec'd binary keeps fd 9
exec 8>&-       # the next runner's check
# the run's memory, booked before it starts (tools/simmem.sh): the DDR array and the model
. ../tools/simmem.sh
simmem_book $(( (1 << MEM_LG2) / 1048576 + 512 )) || exit 1

if [ -n "${CKPT:-}" ]; then
   ../tools/ckpt-stub.py "$CKPT" || exit 1
   echo "=== restoring checkpoint $CKPT ==="
   exec "$BIN" +ckpt="$CKPT" +lram="$CKPT/stub.hex" +cycles=$CYC "$@"
fi
echo "=== booting: fw=$FW dtb=$DTB initrd=${INITRD:-none} a1=$A1 ==="
exec "$BIN" +fw="$FW" +dtb="$DTB" ${INITRD:+ +initrd="$INITRD"} \
     +dtb_off=$OFF_DTB +initrd_off=$OFF_INITRD +a1=$A1 +cycles=$CYC "$@"
