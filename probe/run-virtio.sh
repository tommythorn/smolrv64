#!/bin/bash
# Boot Linux on soc_top with a file-backed virtio-blk device (tb_virtio.v). Reuses the
# existing virtio_blk.v + virtio_mmio.v + the SpiSdCard model (sd_dpi.cpp DPI bridge);
# the device DMAs DIRECTLY into the behavioral DDR (bypassing the D$) = non-coherent DMA
# the kernel handles with Svpbmt NC rings + Zicbom CMO. Resets straight to OpenSBI with
# a1=DTB (rf_shard +a1= seed). Builds the one verilated binary once, then runs it.
#   ./run-virtio.sh [class]          env: CYC, FW, DTB, INITRD, DISK, A1, BUILD=1
set -u
cd "$(dirname "$0")"

# Default workload = the modern Ubuntu 25.04 RISC-V image: a recent kernel that actually
# advertises/uses Zicbom CMO + Svpbmt NC rings, so the non-coherent virtio-blk DMA path is
# exercised for real. ubuntu.dtb already carries the virtio@10002000 node (dma-noncoherent,
# irq 11) + root=/dev/vda1, and Ubuntu mounts vda1 directly with built-in drivers (no initrd).
U=../workloads/ubuntu
FW=${FW:-$U/fw_payload.bin}
DTB=${DTB:-$U/ubuntu.dtb}
INITRD=${INITRD:-}                     # Ubuntu mounts root=/dev/vda1 directly; no initrd
DISK=${DISK:-$U/ubuntu-25.04-preinstalled-server-riscv64.img}
A1=${A1:-82000000}                     # DTB loaded at DDR off 0x0200_0000 = guest 0x8200_0000
CYC=${CYC:-2000000000}                 # 2G cyc cap (~100 min @ ~330k cyc/s); Ctrl-C any time
BIN=obj_dir_virtio/tb_virtio

# ---- build once (or on BUILD=1 / missing binary) ----
if [ -n "${BUILD:-}" ] || [ ! -x "$BIN" ]; then
   echo "building $BIN ..."
   srcs=$(ls *.v | grep -vE '^tb_|probe|^flopwrap.v$|^rf_alu.v')
   extra="../src/virtio_blk.v ../src/virtio_mmio.v ../src/sd_spi_host.v ../src/axi_single_beat_master.v \
          ../src/alu.v ../src/smolrv64_sdpram.v fp_unit_stub.sv ../src/smolrv64_plic_arbiter.v"
   GITC=$(git rev-parse --short=8 HEAD 2>/dev/null || echo 0)   # mimpid = truncated HEAD commit
   verilator --binary --timing -j 0 -sv -CFLAGS -I"$(cd ../src && pwd)" \
      -Wno-fatal -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-CASEINCOMPLETE -Wno-UNUSEDSIGNAL \
      -Wno-UNUSEDPARAM -Wno-DECLFILENAME -Wno-TIMESCALEMOD -Wno-UNOPTFLAT -Wno-LATCH \
      -Wno-PINMISSING -Wno-WIDTHCONCAT -Wno-IMPLICIT -I. -I../src \
      "+define+SMOLRV64_GIT_COMMIT=32'h$GITC" \
      --top-module tb -o tb_virtio --Mdir obj_dir_virtio \
      $srcs tb_virtio.v $extra sd_dpi.cpp || exit 1
fi

echo "=== virtio boot (fw=$FW dtb=$DTB disk=$DISK a1=$A1 cycles=$CYC) ==="
INITRD_ARG=""; [ -n "$INITRD" ] && INITRD_ARG="+initrd=$INITRD"
exec "$BIN" +fw="$FW" +dtb="$DTB" $INITRD_ARG +disk="$DISK" +a1="$A1" +cycles="$CYC"
