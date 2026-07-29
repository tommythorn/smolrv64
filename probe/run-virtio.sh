#!/bin/bash
# Boot Linux on soc_top with a file-backed virtio-blk device (tb_virtio.v). Reuses the
# existing virtio_blk.v + virtio_mmio.v + the SpiSdCard model (sd_dpi.cpp DPI bridge);
# the device DMAs DIRECTLY into the behavioral DDR (bypassing the D$) = non-coherent DMA
# the kernel handles with Svpbmt NC rings + Zicbom CMO. Resets straight to OpenSBI with
# a1=DTB (rf_shard +a1= seed). Builds the one verilated binary once, then runs it.
#   ./run-virtio.sh [class]          env: CYC (0 = no cap, DEFAULT), FW, DTB, INITRD, DISK, A1, BUILD=1
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
[ "${DISK:-}" = none ] && DISK=""      # DISK=none -> no media (RAM/initrd root, e.g. ubuntu-mini)
A1=${A1:-82000000}                     # DTB loaded at DDR off 0x0200_0000 = guest 0x8200_0000
CYC=${CYC:-0}                          # DEFAULT = NO CAP (tb: +cycles=0 = run forever; Ctrl-C /
                                       # kill when done). CYC=N for a finite cap. A 2G default
                                       # once silently expired an interactive boot at [c=1999...].
BIN=obj_dir_virtio/tb_virtio
VDEFS=${VDEFS:-}                                              # extra verilator defines, e.g. -DPROBE_IW=1
[ -n "${PROBE_IW:-}" ] && VDEFS="$VDEFS -DPROBE_IW=$PROBE_IW" # PROBE_IW=N convenience (matches the Makefile)
STAMP=obj_dir_virtio/.built_vdefs                            # the defines the current binary was built with

# ---- build on BUILD=1 / missing binary / changed VDEFS / any source newer than the binary ----
# (the old existence-only check silently ran stale RTL/tb; same fix as run-cosim-linux.sh)
if [ -n "${BUILD:-}" ] || [ ! -x "$BIN" ] || [ "$(cat "$STAMP" 2>/dev/null)" != "$VDEFS" ] || \
   find . ../src -maxdepth 1 \( -name '*.v' -o -name '*.sv' -o -name '*.cpp' -o -name '*.f' \) \
        -newer "$BIN" -print -quit 2>/dev/null | grep -q .; then
   echo "building $BIN (VDEFS='${VDEFS:-<none; PROBE_IW defaults to 2>}') ..."
   srcs=$(ls *.v | grep -vE '^tb_|probe|^flopwrap.v$|^rf_alu.v')
   # REAL CVFPU, not fp_unit_stub: the stub has no conversions (FCVT falls through to
   # rr=ra => F2I returns the RAW FLOAT BITS as the int result) and treats every op as
   # double -- ubuntu userspace FP (glibc/gnulib hash sizing via fdiv.s+fcvt.lu.s) gets
   # silent garbage that WEDGES the boot in the systemd-generator window, masquerading
   # as the FPGA hang. Same file set as run-vl-tests.sh.
   extra="../src/virtio_blk.v ../src/virtio_mmio.v ../src/sd_spi_host.v ../src/axi_single_beat_master.v \
          ../src/alu.v ../src/smolrv64_sdpram.v ../src/smolrv64_plic_arbiter.v \
          -f ../src/cvfpu_sources.f ../src/smolrv64_cvfpu.sv fp_unit.sv"
   GITC=$(git rev-parse --short=8 HEAD 2>/dev/null || echo 0)   # mimpid = truncated HEAD commit
   verilator --binary --timing -j 0 -sv -CFLAGS -I"$(cd ../src && pwd)" \
      -Wno-fatal -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-CASEINCOMPLETE -Wno-UNUSEDSIGNAL \
      -Wno-UNUSEDPARAM -Wno-DECLFILENAME -Wno-TIMESCALEMOD -Wno-UNOPTFLAT -Wno-LATCH \
      -Wno-PINMISSING -Wno-WIDTHCONCAT -Wno-IMPLICIT -I. -I../src \
      "+define+SMOLRV64_GIT_COMMIT=32'h$GITC" $VDEFS \
      --top-module tb -o tb_virtio --Mdir obj_dir_virtio \
      $srcs tb_virtio.v $extra sd_dpi.cpp ckpt_dpi.cpp || exit 1
   echo "$VDEFS" > "$STAMP"
fi

echo "=== virtio boot (fw=$FW dtb=$DTB disk=$DISK a1=$A1 cycles=$CYC defs='$(cat "$STAMP" 2>/dev/null)') ==="
CKPT_ARG=""; [ -n "${CKPT:-}" ] && CKPT_ARG="+ckpt=$CKPT +ckpt_cmd=${CKPT_CMD:-/tmp/probe-ckpt-cmd}"  # fork-checkpoint server
[ -n "${WATCH_VAL:-}" ] && CKPT_ARG="$CKPT_ARG +watch_val=$WATCH_VAL"  # from-reset store-value fingerprint watch
INITRD_ARG=""; [ -n "$INITRD" ] && INITRD_ARG="+initrd=$INITRD"
DISKRO_ARG=""; [ -n "${DISK:-}" ] && [ -z "${DISK_RW:-}" ] && DISKRO_ARG="+disk_ro"   # snapshot by default
DISK_ARG=""; [ -n "${DISK:-}" ] && DISK_ARG="+disk=$DISK"
IOFF_ARG=""; [ -n "${INITRD_OFF:-}" ] && IOFF_ARG="+initrd_off=$INITRD_OFF"
exec "$BIN" +fw="$FW" +dtb="$DTB" $INITRD_ARG $IOFF_ARG $DISK_ARG $DISKRO_ARG $CKPT_ARG +a1="$A1" +cycles="$CYC"
