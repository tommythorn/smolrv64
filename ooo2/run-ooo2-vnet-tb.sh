#!/usr/bin/env bash
# The virtio-net backend under a directed bench (src/tb_virtio_net.v): one TX frame and one RX
# frame through the real DMA engine against a DDR-like AXI memory, every byte and the used
# rings checked, cycles per frame printed. Seconds. Plan item 7 (2026-09-05).
#   ./run-ooo2-vnet-tb.sh        # VNET-TB PASS tx=... rx=...
set -euo pipefail
cd "$(dirname "$0")"
verilator --binary --timing -j 0 -sv -Wall -Wno-fatal \
   -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC \
   -Wno-TIMESCALEMOD -Wno-PROCASSINIT -Wno-BLKSEQ -Wno-PINCONNECTEMPTY -Wno-CASEINCOMPLETE \
   --top-module tb --Mdir obj_dir_vnet -o tb_vnet ../src/virtio_net.v ../src/axi_single_beat_master.v ../src/tb_virtio_net.v \
   > /tmp/vnettb_build.log 2>&1 || { echo "BUILD FAILED"; grep -E '%Error' /tmp/vnettb_build.log | head; exit 1; }
./obj_dir_vnet/tb_vnet 2>&1 | grep -E 'FAIL|PASS|VNET-TB'
