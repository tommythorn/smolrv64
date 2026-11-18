#!/bin/bash
# Build and run every unit testbench under src/ (tb_*.v, and the C++-driven ones: the SD-SPI
# host and virtio_blk against the behavioural SD card, and the Ethernet TX engine through both
# MACs) with Verilator, in parallel, and count PASS. These cover the blocks the core shares (aligner, fetch, decode, ALU, MMU, mul/div, FPU
# wrapper) and the SoC devices (CLINT, PLIC, DDR line bridge, virtio-net). The core's own benches
# live in core/run-*-tb.sh. Rule G4: run this after any port change to a shared module.
#
# Each bench gets every src/ module and its own obj_dir_src_<bench>; --top-module elaborates
# what the bench reaches. Build logs: obj_dir_src_<bench>/build.log, run output: .../run.log.
set -u
cd "$(dirname "$0")"
ulimit -v $((25 * 1024 * 1024)) 2>/dev/null || true   # cap @25 GiB: runaway aborts, not OOM
export SRCS=$(. ./rtl-sources.sh; rtl_sources)

one() {
   local tb=$1 name=${1%.v} d=obj_dir_src_${1%.v} top
   top=$(grep -m1 -oE '^module +[A-Za-z_0-9]+' "$tb" | awk '{print $2}')
   mkdir -p "$d"
   if ! timeout 600 verilator --binary --timing -j 4 -sv -Wno-fatal -Wno-lint -Wno-style -I. \
          --top-module "$top" --Mdir "$d" -o "$name" $SRCS "$tb" fp_unit_stub.sv > "$d/build.log" 2>&1; then
      printf "%-22s BUILD-FAIL (%s/build.log)\n" "$tb" "$d"; return
   fi
   timeout 300 "./$d/$name" > "$d/run.log" 2>&1
   # Each bench prints its own verdict line; a PASS with no FAIL anywhere in the output is a pass.
   if grep -qiE 'ALL TESTS PASSED|ALL 65536 MATCH|(^|[: >])PASS\b' "$d/run.log" && ! grep -qE '\bFAIL' "$d/run.log"; then
      printf "%-22s PASS\n" "$tb"
   else
      printf "%-22s FAIL (%s/run.log)\n" "$tb" "$d"
   fi
}
export -f one
# A C++-driven bench: a Verilated model and its harness, built as the harness's header says;
# the verdict is its "<top>: PASS (0 failure(s))" line.
cxx() {
   local name=$1 top=$2 d=obj_dir_src_$1; shift 2
   mkdir -p "$d"
   if ! timeout 900 verilator --cc --exe --build -j 4 --Mdir "$d" -I. -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNUSEDSIGNAL \
          --top-module "$top" "$@" > "$d/build.log" 2>&1; then
      printf "%-22s BUILD-FAIL (%s/build.log)\n" "$name" "$d"; return
   fi
   timeout 600 "./$d/V$top" > "$d/run.log" 2>&1
   if grep -q 'PASS (0 failure' "$d/run.log" && ! grep -qE '\bFAIL' "$d/run.log"; then
      printf "%-22s PASS\n" "$name"
   else
      printf "%-22s FAIL (%s/run.log)\n" "$name" "$d"
   fi
}

res=$( { ls tb_*.v | xargs -P 8 -I{} bash -c 'one {}';
         cxx sd_spi_host_tb sd_spi_host -GSLOW_HALF=3 -GFAST_HALF=4 -GINIT_BYTES=4 sd_spi_host.v sd_spi_host_tb.cpp;
         cxx virtio_blk_tb virtio_blk -GSD_SLOW_HALF=3 -GSD_FAST_HALF=4 -GSD_INIT_TICKS=20 \
             virtio_blk.v sd_spi_host.v dma_master.v virtio_blk_tb.cpp;
         cxx eth_tx_engine_tb eth_tx_engine_loop_top eth_tx_engine_loop_top.v eth_tx_engine.v \
             eth_mac_tx.v eth_mac_rx.v crc32_d8.v eth_tx_engine_tb.cpp; } | sort)
echo "$res" | grep -v ' PASS$'
echo "----"
echo "tb pass=$(echo "$res" | grep -c ' PASS$') / $(echo "$res" | grep -c .)"
[ "$(echo "$res" | grep -c ' PASS$')" = "$(echo "$res" | grep -c .)" ]   # the status: every bench passed
