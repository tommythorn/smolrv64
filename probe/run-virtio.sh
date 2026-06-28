#!/bin/bash
# Boot Linux on soc_top with a file-backed virtio-blk device (tb_virtio.v). Reuses the
# existing virtio_blk.v + virtio_mmio.v + the SpiSdCard model (sd_dpi.cpp DPI bridge);
# the device DMAs DIRECTLY into the behavioral DDR (bypassing the D$) = non-coherent DMA
# the kernel handles with Svpbmt NC rings + Zicbom CMO. Resets straight to OpenSBI with
# a1=DTB (rf_shard +a1= seed). Builds the one verilated binary once, then runs it.
#   ./run-virtio.sh [class]          env: CYC, FW, DTB, INITRD, DISK, A1, BUILD=1
set -u
cd "$(dirname "$0")"

W=../workloads/linux
FW=${FW:-$W/fw_payload.bin}
DTB=${DTB:-$W/dts-virtio.dtb}          # the virtio-enabled DTB (virtio@10002000 status=okay + zicbom)
INITRD=${INITRD:-$W/tiny128.cpio}
DISK=${DISK:-$W/rootfs.img}
A1=${A1:-82000000}
CYC=${CYC:-160000000}
BIN=obj_dir_virtio/tb_virtio

# ---- build once (or on BUILD=1 / missing binary) ----
if [ -n "${BUILD:-}" ] || [ ! -x "$BIN" ]; then
   echo "building $BIN ..."
   srcs=$(ls *.v | grep -vE '^tb_|probe|^flopwrap.v$|^rf_alu.v')
   extra="../src/virtio_blk.v ../src/virtio_mmio.v ../src/sd_spi_host.v ../src/axi_single_beat_master.v \
          ../src/alu.v ../src/smolrv64_sdpram.v fp_unit_stub.sv ../src/smolrv64_plic_arbiter.v"
   verilator --binary --timing -j 0 -sv -CFLAGS -I"$(cd ../src && pwd)" \
      -Wno-fatal -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-CASEINCOMPLETE -Wno-UNUSEDSIGNAL \
      -Wno-UNUSEDPARAM -Wno-DECLFILENAME -Wno-TIMESCALEMOD -Wno-UNOPTFLAT -Wno-LATCH \
      -Wno-PINMISSING -Wno-WIDTHCONCAT -Wno-IMPLICIT -I. -I../src \
      --top-module tb -o tb_virtio --Mdir obj_dir_virtio \
      $srcs tb_virtio.v $extra sd_dpi.cpp || exit 1
fi

echo "=== virtio boot (fw=$FW dtb=$DTB disk=$DISK a1=$A1 cycles=$CYC) ==="
exec "$BIN" +fw="$FW" +dtb="$DTB" +initrd="$INITRD" +disk="$DISK" +a1="$A1" +cycles="$CYC"
