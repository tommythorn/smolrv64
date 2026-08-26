#!/bin/bash
# Linux-boot lockstep: the in-order SoC vs simmerv. Builds tb_ooo2_linux.v with
# -DINO_COSIM (ooo2_core emits its probe_retire() stream) linked against
# ../src/probe_cosim.cpp + libsimmerv_cosim.a, resets to OpenSBI (0x8000_0000)
# with a1=DTB, and locksteps every retired instruction -- aborting on the first
# divergence with a 320-deep DUT/REF history ring.
#
# This is the tool for the /init stall (docs/ooo2-plan.md): it names the first
# architecturally wrong instruction instead of inferring from a spin address.
#
#   ./run-ooo2-cosim-linux.sh              default tiny128 initrd workload
#   CYC=0 ./run-ooo2-cosim-linux.sh        unbounded (wrap in `timeout`)
#   BUILD=1 ./run-ooo2-cosim-linux.sh      force a rebuild
#   FW=... DTB=... INITRD=... OFF_DTB=... OFF_INITRD=... A1=... MEM_LG2=...
#
# MEM_LG2 sizes THREE things that must agree: the RTL DDR array (OOO2_MEM_SIZE_LG2),
# the C-side bound + simmerv's memory (COSIM_MEM_SIZE_LG2), and ooo2_core's valid-DRAM
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
BIN=$(pwd)/obj_dir_ooo2_clinux/tb_ooo2_clinux

# ALWAYS ask cargo -- do NOT test for the file's existence.  A stale libsimmerv_cosim.a
# silently relinks against old REF behaviour, and that has now cost two hunts: a stale
# probe_cosim.o gave REF and DUT different RAM sizes (91d75614), and a stale .a hid the
# cbo.zero capture fix and reproduced a "MISMATCH" that was already fixed.  Cargo is
# incremental; this costs ~2 s when up to date.
echo "building simmerv cosim lib ..."
(cd "$SIMMERV_DIR" && cargo build --release -p simmerv-cosim) || exit 1
# Relink if the archive is newer than the binary -- verilator's make does not track it.
if [ -f "$BIN" ] && [ "$SIMMERV_LIB" -nt "$BIN" ]; then
   echo "simmerv lib is newer than $BIN -- forcing a rebuild"
   rm -rf obj_dir_ooo2_clinux
fi

PROBE_SRCS="../src/fetch.v ../src/aligner.v ../src/rvc_expand.v \
            ../src/decode_slot.v ../src/decode_operands.v ../src/decode_exec.v \
            ../src/decode_fp.v ../src/predictor.v ../src/exec_alu.v \
            ../src/branch_unit.v ../src/mul3.v ../src/divider.v \
            ../src/csr_file.v ../src/mmu.v ../src/fp_unit.sv \
            rv_cache.v rv_l2_arbiter.v ../src/clint.v ../src/plic.v \
            ../src/ddr_hpm.v"

# STALE-BUILD GUARD.  MEM_LG2 and VDEFS are compile-time -D's, and the C side gets
# MEM_LG2 via -CFLAGS.  `BUILD=1` re-runs verilator, but the generated make then compares
# TIMESTAMPS, not flags -- so probe_cosim.cpp, unchanged on disk, is NOT recompiled and
# keeps the MEM_BYTES from whatever the previous run used.  On 2026-08-20 that gave the
# reference model 512 MiB while the RTL had 2 GiB: every GB5 cosim run "diverged" at
# retire #6979942 on a load to a PA that was real memory to the DUT and off the end of
# the world to simmerv.  Hours went into hunting a core bug that did not exist.
# (Out-of-RAM stores are made inert by cosim_inert_devstore, so the mismatch was
# invisible on the way in and only detectable on the way out -- which pointed the hunt
# at the store path, exactly the wrong place.)
#
# So: record the compile-time config, and when it changes WIPE THE OBJECT DIRECTORY.
# A stamp alone is not enough -- the whole point is that make will not redo the work.
STAMP="obj_dir_ooo2_clinux/.config-stamp"
want="MEM_LG2=$MEM_LG2 VDEFS=${VDEFS:-}"
need_build=0
if [ ! -x "$BIN" ] || [ "${BUILD:-0}" = 1 ]; then
   need_build=1
elif [ "$(cat "$STAMP" 2>/dev/null)" != "$want" ]; then
   need_build=1
   echo "cosim config changed ($want) -> full rebuild"
   rm -rf obj_dir_ooo2_clinux
fi
# Even on BUILD=1, a config difference means stale objects: wipe rather than trust make.
if [ "$need_build" = 1 ] && [ -d obj_dir_ooo2_clinux ] \
   && [ "$(cat "$STAMP" 2>/dev/null)" != "$want" ]; then
   echo "cosim config differs from the built objects -> wiping obj_dir_ooo2_clinux"
   rm -rf obj_dir_ooo2_clinux
fi

if [ "$need_build" = 1 ]; then
   echo "building obj_dir_ooo2_clinux/tb_ooo2_clinux (MEM_LG2=$MEM_LG2) ..."
   verilator --binary --timing -j 0 -sv -Wall \
      -Wno-fatal -Wno-TIMESCALEMOD -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC \
      -Wno-CASEINCOMPLETE -Wno-LATCH -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM -Wno-DECLFILENAME \
      -Wno-ASCRANGE -Wno-UNSIGNED -Wno-WIDTH -Wno-UNOPTFLAT \
      -DINO_COSIM -DINO_MEM_SIZE_LG2=$MEM_LG2 -DCOSIM_MEM_SIZE_LG2=$MEM_LG2 ${VDEFS:-} \
      -CFLAGS "-O2 -I$SIMMERV_INC -DCOSIM_MEM_SIZE_LG2=$MEM_LG2" \
      -LDFLAGS "$SIMMERV_LIB -lpthread -ldl -lm $EXTRA_LD" \
      -I. -I../probe -I../src --top-module tb --Mdir obj_dir_ooo2_clinux -o tb_ooo2_clinux \
      rv_soc_top.v ooo2_core.v ooo2_pending.v ooo2_frontend.v ooo2_predictor.v ooo2_exec.v ooo2_lsu.v rv_regfile.v \
      $PROBE_SRCS ../src/alu.v ../src/smolrv64_sdpram.v ../src/smolrv64_plic_arbiter.v \
      -f ../src/cvfpu_sources.f ../src/smolrv64_cvfpu.sv \
      tb_ooo2_linux.v ../src/probe_cosim.cpp > /tmp/ooo2clinuxbuild.log 2>&1
   if [ $? -ne 0 ]; then echo "BUILD FAILED:"; grep -E '%Error' /tmp/ooo2clinuxbuild.log | head -20; exit 1; fi
   printf '%s' "$want" > "$STAMP"
fi

echo "=== cosim-linux: fw=$FW dtb=$DTB initrd=${INITRD:-none} a1=$A1 mem=2^$MEM_LG2 ==="

# Not `exec`: the run's THROUGHPUT is checked below. Correctness is checked continuously by
# the lockstep, but a change can be perfectly correct and quietly slower, and that has
# happened three times -- see cosim-expected.txt. The retire count at a fixed cycle budget is
# the only instrument that sees it.
set -o pipefail
"$BIN" +fw="$FW" +dtb="$DTB" ${INITRD:+ +initrd="$INITRD"} \
     +dtb_off=$OFF_DTB +initrd_off=$OFF_INITRD +a1=$A1 +cycles=$CYC 2>&1 | tee /tmp/ooo2-cosim.out
rc=$?

hw=$(printf '%s' "${VDEFS:-}" | sed -n 's/.*-DINO_HW=\([0-9]*\).*/\1/p'); hw=${hw:-2}
got=$(sed -n 's/.*TIMEOUT after [0-9]* cycles (retires=\([0-9]*\).*/\1/p' /tmp/ooo2-cosim.out | tail -1)
exp=$(awk -v c="$CYC" -v h="$hw" '!/^#/ && NF>=4 && $1==c && $2==h {print $3; exit}' cosim-expected.txt)
tol=$(awk -v c="$CYC" -v h="$hw" '!/^#/ && NF>=4 && $1==c && $2==h {print $4; exit}' cosim-expected.txt)

if [ -n "$got" ] && [ -n "$exp" ]; then
   floor=$(awk -v e="$exp" -v t="$tol" 'BEGIN{printf "%d", e*(100-t)/100}')
   pct=$(awk -v g="$got" -v e="$exp" 'BEGIN{printf "%+.2f", 100*(g-e)/e}')
   if [ "$got" -lt "$floor" ]; then
      echo "COSIM-PERF FAIL: retires=$got vs expected $exp ($pct%, floor $floor at ${tol}%)"
      echo "  A correct-but-slower change. If it is intended, raise the number in"
      echo "  ooo2/cosim-expected.txt in the SAME commit, with the reason."
      exit 1
   fi
   echo "cosim-perf: retires=$got vs expected $exp ($pct%) -- ok"
elif [ -n "$got" ]; then
   echo "cosim-perf: retires=$got (no expectation recorded for CYC=$CYC OOO2_HW=$hw)"
fi
exit $rc
