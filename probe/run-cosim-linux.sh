#!/bin/bash
# Linux-boot lockstep: soc_top vs simmerv. Builds tb_cosim_linux (soc_top + caches +
# CLINT/PLIC/UART + behavioral DDR) with -DPROBE_COSIM, resets to OpenSBI (0x8000_0000)
# with a1=DTB, and locksteps every committed instruction against simmerv -- aborts on the
# first divergence (the high-signal output for an overnight run).
#
# Defaults run workloads/linux (tiny128). Override via env for other workloads -- the
# gb5/gb6 dirs ship a `make pcosim` that sets these:
#   NAME       per-workload binary/obj-dir tag (default linux)
#   MEM_LG2    log2(DDR bytes); sizes the RTL ram[], the C bound, AND simmerv (default 28=256MiB)
#   FW DTB INITRD   image paths (absolute; this script cd's to probe/)
#   OFF_DTB OFF_INITRD   load offsets from 0x8000_0000, hex no-0x (default linux 2000000 / 762b000)
#   A1         DTB physical address seeded into a1, hex no-0x (default 82000000)
#   CYC        cycle cap (default 200000000);  BUILD=1 forces a rebuild
set -u
cd "$(dirname "$0")"

SIMMERV_DIR=${SIMMERV_DIR:-$HOME/simmerv}
SIMMERV_LIB=$SIMMERV_DIR/target/release/libsimmerv_cosim.a
SIMMERV_INC=$SIMMERV_DIR/cosim

NAME=${NAME:-linux}
MEM_LG2=${MEM_LG2:-28}
W=../workloads/linux
FW=${FW:-$W/fw_payload.bin}; DTB=${DTB:-$W/dts.dtb}; INITRD=${INITRD:-$W/tiny128.cpio}
OFF_DTB=${OFF_DTB:-2000000}; OFF_INITRD=${OFF_INITRD:-762b000}; A1=${A1:-82000000}
CYC=${CYC:-200000000}
BIN=$(pwd)/obj_dir_cosim_${NAME}/tb_cosim_${NAME}
STAMP=$(pwd)/obj_dir_cosim_${NAME}/.build_stamp   # records the compile-time config baked in

[ -f "$SIMMERV_LIB" ] || (cd "$SIMMERV_DIR" && cargo build --release -p simmerv-cosim) || exit 1

# On macOS the simmerv static archive pulls in a vmnet shim; link the framework
# (cargo's build.rs adds this for simmerv's own binaries, but a .a can't carry it).
OSLIBS=
[ "$(uname -s)" = Darwin ] && OSLIBS="-framework vmnet"

# Decide whether to (re)build. The old check keyed ONLY on binary existence, so a
# stale binary silently ran old RTL -- and MEM_LG2/VDEFS are compile-time -D's, so a
# size change (e.g. gb5 1->2 GiB) was inert until a manual BUILD=1. Now rebuild when:
#   - the binary is missing, or BUILD=1, or
#   - the compile-time config (MEM_LG2 + VDEFS) differs from what's baked in, or
#   - any source under probe/ or ../src/ is newer than the binary.
# (../src is scanned at maxdepth 1; the stable cvfpu subtree is intentionally excluded.)
want="MEM_LG2=$MEM_LG2 VDEFS=${VDEFS:-}"
need_build=0
if [ ! -x "$BIN" ] || [ "${BUILD:-0}" = 1 ]; then need_build=1
elif [ "$(cat "$STAMP" 2>/dev/null)" != "$want" ]; then need_build=1; echo "config changed ($want) -> rebuild"
elif find . ../src -maxdepth 1 \( -name '*.v' -o -name '*.sv' -o -name '*.vh' -o -name '*.cpp' -o -name '*.f' \) \
        -newer "$BIN" -print -quit 2>/dev/null | grep -q .; then
   need_build=1; echo "source newer than binary -> rebuild"
fi

if [ "$need_build" = 1 ]; then
   srcs=$(ls *.v | grep -vE '^tb_|probe|^flopwrap.v$|^rf_alu.v')
   echo "building obj_dir_cosim_${NAME}/tb_cosim_${NAME} (MEM_LG2=$MEM_LG2) ..."
   verilator --binary --timing -j 0 -sv -Wall \
      -Wno-fatal -Wno-TIMESCALEMOD -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC \
      -Wno-CASEINCOMPLETE -Wno-LATCH -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM -Wno-DECLFILENAME \
      -Wno-ASCRANGE -Wno-UNSIGNED -Wno-WIDTH -Wno-UNOPTFLAT \
      -DPROBE_COSIM -DCOSIM_MEM_SIZE_LG2=$MEM_LG2 ${VDEFS:-} \
      -CFLAGS "-O2 -DCOSIM_MEM_SIZE_LG2=$MEM_LG2 -I$SIMMERV_INC" \
      -LDFLAGS "$SIMMERV_LIB -lpthread -ldl -lm $OSLIBS" \
      -I. -I../src --top-module tb --Mdir obj_dir_cosim_${NAME} -o tb_cosim_${NAME} \
      $srcs tb_cosim_linux.v ../src/alu.v ../src/smolrv64_sdpram.v -f ../src/cvfpu_sources.f ../src/smolrv64_cvfpu.sv \
      fp_unit.sv ../src/smolrv64_plic_arbiter.v probe_cosim.cpp > /tmp/cosim_${NAME}_build.log 2>&1
   if [ $? -ne 0 ]; then echo "BUILD FAILED:"; grep -E '%Error' /tmp/cosim_${NAME}_build.log | head; exit 1; fi
   echo "$want" > "$STAMP"
fi

echo "=== cosim '$NAME' (mem=$((1<<(MEM_LG2-20)))MiB fw=$FW dtb=$DTB@+$OFF_DTB initrd=$INITRD@+$OFF_INITRD a1=$A1) ==="
"$BIN" +fw="$FW" +dtb="$DTB" +initrd="$INITRD" \
       +a1=$A1 +dtb_off=$OFF_DTB +initrd_off=$OFF_INITRD +cycles=$CYC
