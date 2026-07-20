#!/bin/bash
# Linux-boot lockstep: soc_top vs simmerv. Builds tb_cosim_linux (soc_top + caches +
# CLINT/PLIC/UART + behavioral DDR) with -DPROBE_COSIM, resets to OpenSBI (0x8000_0000)
# with a1=DTB, and locksteps every committed instruction against simmerv -- aborts on the
# first divergence (the high-signal output for an overnight run).
#
# Defaults = the GOLDEN 6.x config that boots to login: workloads/ubuntu/fw_payload.bin
# (OpenSBI + the Ubuntu 6.x kernel -- the ONLY blessed kernel; the old workloads/linux
# 5.4 payload dies with "FATAL: kernel too old" against the tiny128 userspace) +
# tiny128 rootfs + workloads/tiny128/tiny128-cosim.dtb (virtio-free, honest timebase).
# Override via env for other workloads -- the gb5/gb6 dirs ship a `make pcosim`:
#   NAME       per-workload binary/obj-dir tag (default linux)
#   MEM_LG2    log2(DDR bytes); sizes the RTL ram[], the C bound, AND simmerv (default 29=512MiB)
#   FW DTB INITRD   image paths (absolute; this script cd's to probe/)
#   OFF_DTB OFF_INITRD   load offsets from 0x8000_0000, hex no-0x (default 1ff00000 / 1f52c000)
#   A1         DTB physical address seeded into a1, hex no-0x (default 9ff00000)
#   CYC        cycle cap (default 2000000000000);  BUILD=1 forces a rebuild
set -u
cd "$(dirname "$0")"

SIMMERV_DIR=${SIMMERV_DIR:-$HOME/simmerv}
SIMMERV_LIB=$SIMMERV_DIR/target/release/libsimmerv_cosim.a
SIMMERV_INC=$SIMMERV_DIR/cosim

NAME=${NAME:-linux}
THREADS=${THREADS:-1}   # verilator --threads. MEASURED 2026-07-06: 4 threads is ~2.4x SLOWER
                        # (622s -> 1505s per 100M cyc; per-retire DPI + timing coroutines
                        # partition badly). Streams bit-identical. Keep 1.
MEM_LG2=${MEM_LG2:-29}
W=../workloads/tiny128
FW=${FW:-../workloads/ubuntu/fw_payload.bin}; DTB=${DTB:-$W/tiny128-cosim.dtb}; INITRD=${INITRD:-$W/tiny128.cpio}
OFF_DTB=${OFF_DTB:-1ff00000}; OFF_INITRD=${OFF_INITRD:-1f52c000}; A1=${A1:-9ff00000}
CYC=${CYC:-2000000000000}
# STRESS=1 = the sandbox-stress run (systemd generator-child stall chase). The stress cpio
# is bigger than the flush-packed golden slot (initrd-end == DTB base), so it loads LOWER
# with its own dtb (tiny128-cosim-stress.dts: initrd @ 0x9f400000). Overrides the above.
if [ -n "${STRESS:-}" ]; then
   NAME=stress; INITRD=$W/tiny128-stress.cpio; DTB=$W/tiny128-cosim-stress.dtb
   OFF_INITRD=1f400000
fi
# Refuse overlapping initrd/dtb load regions. The initrd loads LAST, so an oversized cpio
# at the golden offset silently clobbers the DTB -- and OpenSBI parses the FDT before it
# brings up the console (uart addr comes FROM the dtb), so the boot dies with NO output.
# Bit twice with tiny128-stress.cpio + the default layout; STRESS=1 is the supported form.
# UBUNTU=1 = the disk-root Ubuntu boot UNDER THE ORACLE (the systemd-generator
# corruption chase): 2 GiB, no initrd, virtio-blk backed by DISK (default = the
# repro image). The oracle diverges at the FIRST architecturally-wrong load --
# hours before GLib's g_hash_table probe loop ever spins on the corruption.
if [ -n "${UBUNTU:-}" ]; then
   NAME=ubuntu; MEM_LG2=31
   # ubuntu-cosim.dtb = ubuntu.dts minus the virtio-net node: the sim has no net backend
   # and the oracle wants no unbacked-window traffic (DUT faulted at 0x10003000 there).
   FW=../workloads/ubuntu/fw_payload.bin; DTB=${DTB:-../workloads/ubuntu/ubuntu-cosim.dtb}; INITRD=
   OFF_DTB=2000000; A1=82000000
   DISK=${DISK:-$HOME/simmerv/linux/ubuntu-25.04-preinstalled-server-riscv64.img}
fi
# RAM=1 = ubuntu-mini initramfs boot UNDER THE ORACLE: same systemd/GLib/generator stack,
# rootfs entirely in RAM -- no SD, no virtio, no non-coherent DMA (the storage-path
# discriminator). Artifact + dtb from workloads/ubuntu-mini/build-mini.sh.
if [ -n "${RAM:-}" ]; then
   NAME=ram; MEM_LG2=31
   FW=../workloads/ubuntu/fw_payload.bin; DTB=${DTB:-../workloads/ubuntu-mini/ubuntu-ram.dtb}
   # UNCOMPRESSED cpio for sim/cosim (zstd decompress = ~16B insns, impractical in RTL);
   # the .zst is for FPGA (serial-upload dominated). Override via INITRD=.
   INITRD=../workloads/ubuntu-mini/ubuntu-mini.cpio
   OFF_DTB=2000000; OFF_INITRD=10000000; A1=82000000; DISK=
fi
if [ -n "${INITRD:-}" ] && [ -f "$INITRD" ] && [ -f "$DTB" ]; then
   isz=$(wc -c < "$INITRD"); dsz=$(wc -c < "$DTB")
   i0=$((16#$OFF_INITRD)); d0=$((16#$OFF_DTB))
   if [ "$((i0 < d0 + dsz && d0 < i0 + isz))" = 1 ]; then
      echo "ERROR: initrd [0x$OFF_INITRD +$isz) overlaps dtb [0x$OFF_DTB +$dsz)." >&2
      echo "       (oversized cpio? use STRESS=1, or set OFF_INITRD/OFF_DTB explicitly)" >&2
      exit 1
   fi
fi
BIN=$(pwd)/obj_dir_cosim_${NAME}/tb_cosim_${NAME}
STAMP=$(pwd)/obj_dir_cosim_${NAME}/.build_stamp   # records the compile-time config baked in

# Always cargo-build (no-op when clean, ~0.2s): the old existence-only check silently
# ran a STALE lib after simmerv source changes -- same footgun class as the binary check.
(cd "$SIMMERV_DIR" && cargo build --release -p simmerv-cosim) || exit 1

# OS-specific link libs. Linux needs -ldl (dlopen) for the Rust static archive;
# macOS has no libdl (dlopen is in libSystem) and instead needs the vmnet framework
# the simmerv .a pulls in (cargo's build.rs adds it for simmerv's own binaries, but
# a .a can't carry it).
if [ "$(uname -s)" = Darwin ]; then OSLIBS="-framework vmnet"; else OSLIBS="-ldl"; fi

# PERF_TRACE=1 builds the performance event trace in (backend_top.v taps + perf_trace.cpp
# DPI sink). The sink is env-configured at run time: PERF_TRACE_OUT (file) and
# PERF_TRACE_WIN="start,len" (cycle window -- ESSENTIAL for a long run like gb5, or the
# trace fills the disk). Folded into the build stamp so toggling it forces a rebuild.
PERFOPT=""; PERFSRC=""
[ -n "${PERF_TRACE:-}" ] && { PERFOPT="-DPERF_TRACE"; PERFSRC="perf_trace.cpp"; }

# Decide whether to (re)build. The old check keyed ONLY on binary existence, so a
# stale binary silently ran old RTL -- and MEM_LG2/VDEFS are compile-time -D's, so a
# size change (e.g. gb5 1->2 GiB) was inert until a manual BUILD=1. Now rebuild when:
#   - the binary is missing, or BUILD=1, or
#   - the compile-time config (MEM_LG2 + VDEFS) differs from what's baked in, or
#   - any source under probe/ or ../src/ is newer than the binary, or
#   - the simmerv cosim lib (.a) is newer than the binary -- a simmerv-only change rebuilds
#     the .a (cargo, above) but touches no probe/ file, so this is its ONLY relink trigger.
# (../src is scanned at maxdepth 1; the stable cvfpu subtree is intentionally excluded.)
want="MEM_LG2=$MEM_LG2 VDEFS=${VDEFS:-} PERF=${PERF_TRACE:-} THREADS=$THREADS"
need_build=0
if [ ! -x "$BIN" ] || [ "${BUILD:-0}" = 1 ]; then need_build=1
elif [ "$(cat "$STAMP" 2>/dev/null)" != "$want" ]; then need_build=1; echo "config changed ($want) -> rebuild"
elif [ "$SIMMERV_LIB" -nt "$BIN" ]; then need_build=1; echo "simmerv lib newer than binary -> relink (simmerv-only change)"
elif find . ../src -maxdepth 1 \( -name '*.v' -o -name '*.sv' -o -name '*.vh' -o -name '*.cpp' -o -name '*.f' \) \
        -newer "$BIN" -print -quit 2>/dev/null | grep -q .; then
   need_build=1; echo "source newer than binary -> rebuild"
fi

if [ "$need_build" = 1 ]; then
   srcs=$(ls *.v | grep -vE '^tb_|probe|^flopwrap.v$|^rf_alu.v')
   echo "building obj_dir_cosim_${NAME}/tb_cosim_${NAME} (MEM_LG2=$MEM_LG2) ..."
   verilator --binary --timing -j 0 --threads $THREADS -sv -Wall \
      -Wno-fatal -Wno-TIMESCALEMOD -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC \
      -Wno-CASEINCOMPLETE -Wno-LATCH -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM -Wno-DECLFILENAME \
      -Wno-ASCRANGE -Wno-UNSIGNED -Wno-WIDTH -Wno-UNOPTFLAT \
      -DPROBE_COSIM -DCOSIM_MEM_SIZE_LG2=$MEM_LG2 ${VDEFS:-} $PERFOPT \
      -CFLAGS "-O2 -DCOSIM_MEM_SIZE_LG2=$MEM_LG2 -I$SIMMERV_INC -I$(cd ../src && pwd)" \
      -LDFLAGS "$SIMMERV_LIB -lpthread -lm $OSLIBS" \
      -I. -I../src --top-module tb --Mdir obj_dir_cosim_${NAME} -o tb_cosim_${NAME} \
      $srcs tb_cosim_linux.v ../src/alu.v ../src/smolrv64_sdpram.v -f ../src/cvfpu_sources.f ../src/smolrv64_cvfpu.sv \
      fp_unit.sv ../src/smolrv64_plic_arbiter.v \
      ../src/virtio_blk.v ../src/virtio_mmio.v ../src/sd_spi_host.v ../src/axi_single_beat_master.v \
      probe_cosim.cpp sd_dpi.cpp $PERFSRC > /tmp/cosim_${NAME}_build.log 2>&1
   if [ $? -ne 0 ]; then echo "BUILD FAILED:"; grep -E '%Error' /tmp/cosim_${NAME}_build.log | head; exit 1; fi
   echo "$want" > "$STAMP"
fi

echo "=== cosim '$NAME' (mem=$((1<<(MEM_LG2-20)))MiB fw=$FW dtb=$DTB@+$OFF_DTB initrd=${INITRD:-none}@+$OFF_INITRD disk=${DISK:-none} a1=$A1) ==="
# Snapshot by default: sim runs used to WRITE the image (journal replay, systemd state),
# so every run mutated it and copies diverged across machines. DISK_RW=1 opts back in.
DISKRO_ARG=""; [ -n "${DISK:-}" ] && [ -z "${DISK_RW:-}" ] && DISKRO_ARG="+disk_ro"
"$BIN" +fw="$FW" +dtb="$DTB" \
       ${INITRD:+"+initrd=$INITRD"} ${DISK:+"+disk=$DISK"} $DISKRO_ARG \
       +a1=$A1 +dtb_off=$OFF_DTB +initrd_off=$OFF_INITRD +cycles=$CYC \
       ${WATCHPA:+"+watchpa=$WATCHPA"}
