#!/bin/bash
# Linux-boot lockstep: the SoC (rv_soc_top) vs simmerv. Builds tb_ooo2_linux.v with
# -DOOO2_COSIM (ooo2_core emits its probe_retire() stream) linked against
# ../src/probe_cosim.cpp + libsimmerv_cosim.a, resets to OpenSBI (0x8000_0000)
# with a1=DTB, and locksteps every retired instruction -- aborting on the first
# divergence with a 320-deep DUT/REF history ring.
#
# This is the tool for a boot that stalls (docs/history/ooo2-plan.md): it names the first
# architecturally wrong instruction instead of inferring from a spin address.
#
#   ./run-ooo2-cosim-linux.sh              default tiny128 initrd workload
#   CYC=0 ./run-ooo2-cosim-linux.sh        unbounded (wrap in `timeout`)
#   BUILD=1 ./run-ooo2-cosim-linux.sh      force a rebuild
#   FW=... DTB=... INITRD=... OFF_DTB=... OFF_INITRD=... A1=... MEM_LG2=...
#   DISK=<image>   a virtio-blk disk (the DTB must carry the node); the image is read-only
#                  (writes stay in RAM) unless DISK_RW=1. The console must print BLKCHECK-OK
#                  (workloads/tiny128/blkcheck.sh) and never BLKCHECK-FAIL; no retire-count row
#                  applies (a disk boot retires a different count).
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
# THIS run's output, not a shared file: two runs that overlap on one obj_dir (a rule already, and
# broken on 2026-09-17) graded each other's retire count through obj_dir_ooo2_clinux/last-run.out.
RUNOUT=$(mktemp -t ooo2-cosim-run.XXXXXX); trap 'rm -f "$RUNOUT"' EXIT
# ONE BUILD DIRECTORY PER CONFIG (2026-09-17): the default config keeps the plain name (every log
# grep for `building obj_dir_ooo2_clinux` still matches, rule G5); any other MEM_LG2/VDEFS gets
# obj_dir_ooo2_clinux.<hash>, so the IW=2 and IW=3 binaries, the storm and a sweep coexist and run
# in parallel instead of wiping each other. Runs of the SAME config share the binary under a SHARED
# lock and a rebuild takes the lock EXCLUSIVELY (waiting for the runs in flight; the linker cannot
# rewrite a running executable), so two runs of one config proceed together when nothing changed
# and never rebuild under each other when something did: the hand-made queue is the runner's now.
OBJ=obj_dir_ooo2_clinux
if [ "$MEM_LG2" != 29 ] || [ -n "${VDEFS:-}" ]; then
   OBJ=obj_dir_ooo2_clinux.$(printf '%s|%s' "$MEM_LG2" "${VDEFS:-}" | sha1sum | cut -c1-8)
fi
BIN=$(pwd)/$OBJ/tb_ooo2_clinux
mkdir -p "$OBJ"; exec 9>"$OBJ.flock"; flock -s 9

# ALWAYS ask cargo -- do NOT test for the file's existence.  A stale libsimmerv_cosim.a
# silently relinks against old REF behaviour, and that has now cost two hunts: a stale
# probe_cosim.o gave REF and DUT different RAM sizes (91d75614), and a stale .a hid the
# cbo.zero capture fix and reproduced a "MISMATCH" that was already fixed.  Cargo is
# incremental; this costs ~2 s when up to date.
echo "building simmerv cosim lib ..."
(cd "$SIMMERV_DIR" && cargo build --release -p simmerv-cosim) || exit 1
# Relink if the archive is newer than the binary -- verilator's make does not track it.
wipe=0
if [ -f "$BIN" ] && [ "$SIMMERV_LIB" -nt "$BIN" ]; then
   echo "simmerv lib is newer than $BIN -- forcing a rebuild"
   wipe=1
fi

PROBE_SRCS="../src/fetch.v ../src/aligner.v ../src/rvc_expand.v \
            ../src/decode_slot.v ../src/decode_operands.v ../src/decode_exec.v \
            ../src/decode_fp.v ../src/exec_alu.v \
            ../src/branch_unit.v ../src/mul3.v ../src/divider.v \
            ../src/csr_file.v ../src/mmu.v ../src/fp_unit.sv \
            rv_cache.v rv_icache.v rv_errlog.v rv_l2_arbiter.v ../src/clint.v ../src/plic.v \
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
STAMP="$OBJ/.config-stamp"
# G1: echo the config THIS RUN consumed, every run -- not only when it rebuilds. A run that
# reuses a binary is exactly the one whose config you cannot see, and reading it back out of
# .config-stamp after the fact is how you end up trusting a number you did not verify.
# THE STAMP COVERS THE SOURCES TOO. Without this, a run with BUILD unset reused whatever
# binary was there, and on 2026-09-04 a whole day of commits was declared "bit-identical"
# against a model none of them had been compiled into -- every log said "building simmerv
# cosim lib" and none said "building obj_dir_ooo2_clinux". A verdict from a binary that
# does not contain the change is not a verdict. The hash is of every RTL file the model is
# built from -- EVERY .v/.sv under ooo2/ and src/, because Verilator pulls modules in by
# name from the -I paths and a list of the top-level files missed ooo2_lq.v on the first
# try (the hash did not move when the queue changed, 2026-09-04, the same afternoon).
# Any edit forces the rebuild; BUILD=1 is only for a wiped or foreign tree.
srchash=$(cat *.v ../src/*.v ../src/*.sv ../src/probe_cosim.cpp ../src/sd_dpi.cpp $(grep -v '^+\|^$' ../src/cvfpu_sources.f) 2>/dev/null | sha1sum | cut -c1-16)
want="MEM_LG2=$MEM_LG2 VDEFS=${VDEFS:-} SRC=$srchash"
echo "cosim config: MEM_LG2=$MEM_LG2 VDEFS=${VDEFS:-<none>} CYC=${CYC:-<default>} model-src=$srchash"
need_build=0
if [ ! -x "$BIN" ] || [ "${BUILD:-0}" = 1 ]; then
   need_build=1
elif [ "$(cat "$STAMP" 2>/dev/null)" != "$want" ]; then
   need_build=1
   echo "cosim config or sources changed ($want) -> full rebuild"
   wipe=1
fi
# Even on BUILD=1, a config difference means stale objects: wipe rather than trust make.
if [ "$need_build" = 1 ] && [ -f "$STAMP" ] \
   && [ "$(cat "$STAMP" 2>/dev/null)" != "$want" ]; then
   echo "cosim config differs from the built objects -> wiping $OBJ"
   wipe=1
fi
[ "$wipe" = 1 ] && need_build=1

if [ "$need_build" = 1 ]; then
   # upgrade to the exclusive lock (waits for every run of this config in flight), then re-check:
   # the run we waited for may have been the rebuild we needed.
   flock -n -x 9 || { echo "cosim: waiting for the runs of this config in flight before rebuilding $OBJ ..."; flock -x 9; }
   [ "$(cat "$STAMP" 2>/dev/null)" = "$want" ] && [ -x "$BIN" ] && ! [ "$SIMMERV_LIB" -nt "$BIN" ] && need_build=0
   # the wipe happens HERE, under the exclusive lock: never under a run of this config in flight
   [ "$need_build" = 1 ] && rm -rf "$OBJ"
fi
if [ "$need_build" = 1 ]; then
   echo "building $OBJ/tb_ooo2_clinux (MEM_LG2=$MEM_LG2 VDEFS=${VDEFS:-<none>}) ..."
   mkdir -p "$OBJ"
   verilator --binary --timing -j 0 -sv -Wall \
      -Wno-fatal -Wno-TIMESCALEMOD -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC \
      -Wno-CASEINCOMPLETE -Wno-LATCH -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM -Wno-DECLFILENAME \
      -Wno-ASCRANGE -Wno-UNSIGNED -Wno-WIDTH -Wno-UNOPTFLAT \
      -DOOO2_COSIM -DOOO2_MEM_SIZE_LG2=$MEM_LG2 -DCOSIM_MEM_SIZE_LG2=$MEM_LG2 ${VDEFS:-} \
      -CFLAGS "-O2 -I$SIMMERV_INC -DCOSIM_MEM_SIZE_LG2=$MEM_LG2" \
      -LDFLAGS "$SIMMERV_LIB -lpthread -ldl -lm $EXTRA_LD" \
      -I. -I../src --top-module tb --Mdir "$OBJ" -o tb_ooo2_clinux \
      rv_soc_top.v ooo2_core.v ooo2_pending.v ooo2_frontend.v ooo2_predictor.v ooo2_exec.v ooo2_lsu.v rv_regfile.v \
      $PROBE_SRCS ../src/alu.v ../src/smolrv64_sdpram.v ../src/smolrv64_plic_arbiter.v \
      -f ../src/cvfpu_sources.f \
      ../src/virtio_blk.v ../src/virtio_mmio.v ../src/sd_spi_host.v ../src/axi_single_beat_master.v ../src/sd_dpi.cpp \
      tb_ooo2_linux.v ../src/probe_cosim.cpp > "$OBJ/build.log" 2>&1
   if [ $? -ne 0 ]; then echo "BUILD FAILED:"; grep -E '%Error' "$OBJ/build.log" | head -20; exit 1; fi
   printf '%s' "$want" > "$STAMP"
fi
flock -s 9      # back to shared for the run

echo "=== cosim-linux: fw=$FW dtb=$DTB initrd=${INITRD:-none} disk=${DISK:-none} a1=$A1 mem=2^$MEM_LG2 ==="

# Not `exec`: the run's THROUGHPUT is checked below. Correctness is checked continuously by
# the lockstep, but a change can be perfectly correct and quietly slower, and that has
# happened three times -- see cosim-expected.txt. The retire count at a fixed cycle budget is
# the only instrument that sees it.
set -o pipefail
"$BIN" +fw="$FW" +dtb="$DTB" ${INITRD:+ +initrd="$INITRD"} \
     +dtb_off=$OFF_DTB +initrd_off=$OFF_INITRD +a1=$A1 +cycles=$CYC ${PLUSARGS:-} \
     ${DISK:+ +disk="$DISK"} $( [ -n "${DISK:-}" ] && [ -z "${DISK_RW:-}" ] && echo +disk_ro ) 2>&1 | tee "$RUNOUT"
rc=$?
# The disk workload's own verdict: its init mounts the disk, checks every byte, writes a copy
# back and re-reads it past the page cache (BLKCHECK-OK). Absent or FAIL = the run failed,
# whatever the lockstep said: a DMA that lands wrong bytes in BOTH memories is invisible to it.
# The console is written a character at a time and the testbench's `[c=...]` progress line can
# land INSIDE a word ("BLKCHEC[c=1461000000 ...]\nK-OK" on the first 1.5 G run), so the verdict is
# read from the console with those lines removed and the newlines joined.
console() { sed 's/\[c=[^]]*\]//g' "$RUNOUT" | tr -d '\n'; }
if [ -n "${DISK:-}" ]; then
   if console | grep -q 'BLKCHECK-FAIL' || ! console | grep -q 'BLKCHECK-OK'; then
      echo "BLKCHECK FAIL: the disk check did not pass on the console (DISK=$DISK)"; rc=1
   else echo "blkcheck: BLKCHECK-OK on the console (dma beats: $(grep -o 'dma rd=[0-9]* wr=[0-9]*' "$RUNOUT" | tail -1))"; fi
fi

# OOO2_HW, not INO_HW: the define was renamed with the core and this line was not, so it
# reported the DEFAULT width no matter what VDEFS actually set. That is not cosmetic -- the
# sim default (2 = 32-bit fetch) is NOT what the FPGA runs (4 = 64-bit), and at HW=2 the RVC
# aligner stalls 37% of cycles against 8% at HW=4. A whole priority list was built on the
# wrong number before this was caught. Measure at the width the hardware uses.
hw=$(printf '%s' "${VDEFS:-}" | sed -n 's/.*-DOOO2_HW=\([0-9]*\).*/\1/p'); hw=${hw:-8}
# The rows are keyed by cycles and fetch width only; a non-default PIPELINE width (OOO2_IW=3, or 1)
# retires a different count and has no row, so its verdict is the lockstep alone -- comparing it
# against the IW=2 row printed COSIM-PERF FAIL three times on 2026-09-17 for runs that were clean.
iw=$(printf '%s' "${VDEFS:-}" | sed -n 's/.*-DOOO2_IW=\([0-9]*\).*/\1/p'); iw=${iw:-2}
# A row's optional 5th column is the pipeline width it was measured at (2 when absent), so
# IW=3 has its own rows since 2026-09-17 -- and its own count, now that the testbench sums
# the third commit port. A width with no row is judged by the lockstep alone.
got=$(sed -n 's/.*TIMEOUT after [0-9]* cycles (retires=\([0-9]*\).*/\1/p' "$RUNOUT" | tail -1)
# The trailing `# provenance` is stripped before the fields are counted, or the `#` reads as the
# width column and every commented row silently stops grading (it did, for one evening).
exp=$(awk -v c="$CYC" -v h="$hw" -v w="$iw" '{sub(/#.*/,"")} NF>=4 && $1==c && $2==h && (NF>=5 ? $5 : 2)==w {print $3; exit}' cosim-expected.txt)
tol=$(awk -v c="$CYC" -v h="$hw" -v w="$iw" '{sub(/#.*/,"")} NF>=4 && $1==c && $2==h && (NF>=5 ? $5 : 2)==w {print $4; exit}' cosim-expected.txt)
[ -z "$exp" ] && echo "cosim-perf: no expectation row for CYC=$CYC OOO2_HW=$hw OOO2_IW=$iw -- the lockstep is the verdict"

# an expectation that is not a number is no expectation (a placeholder row passed as "ok" once)
[ -n "${DISK:-}" ] && exp=""     # a disk boot has no row: the lockstep and BLKCHECK are its verdict
case "$exp" in ''|*[!0-9]*) exp="";; esac
if [ -n "$got" ] && [ -n "$exp" ]; then
   floor=$(awk -v e="$exp" -v t="$tol" 'BEGIN{printf "%d", e*(100-t)/100}')
   pct=$(awk -v g="$got" -v e="$exp" 'BEGIN{printf "%+.2f", 100*(g-e)/e}')
   if [ "$got" -lt "$floor" ]; then
      echo "COSIM-PERF FAIL: retires=$got vs expected $exp ($pct%, floor $floor at ${tol}%) model-src=$srchash"
      echo "  A correct-but-slower change. If it is intended, raise the number in"
      echo "  ooo2/cosim-expected.txt in the SAME commit, with the reason."
      exit 1
   fi
   echo "cosim-perf: retires=$got vs expected $exp ($pct%) -- ok  model-src=$srchash"
elif [ -n "$got" ]; then
   echo "cosim-perf: retires=$got (no expectation recorded for CYC=$CYC OOO2_HW=$hw) model-src=$srchash"
fi
exit $rc
