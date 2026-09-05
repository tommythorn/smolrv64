#!/usr/bin/env bash
# The Ethernet RX engine under its bench (src/eth_rx_engine_tb.cpp): frames through the real
# TX and RX MACs across independent clocks into the slotted engine, read back and checked; a
# burst, a full ring, an ack landing mid-frame, an FCS-bad frame, an over-long frame. Seconds.
# Named run-ooo2-*-tb.sh so tools/gate.sh picks it up (2026-09-05).
#   ./run-ooo2-ethrx-tb.sh        # eth_rx_engine: PASS
set -euo pipefail
cd "$(dirname "$0")/../src"
verilator --cc --exe --build -j 0 -Wno-fatal \
   -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNUSEDSIGNAL -Wno-DECLFILENAME -Wno-TIMESCALEMOD \
   --Mdir obj_dir_rxeng --top-module eth_rx_engine_loop_top -GSLOT_BYTES=1024 \
   eth_rx_engine_loop_top.v eth_mac_tx.v eth_mac_rx.v eth_rx_engine.v crc32_d8.v eth_rx_engine_tb.cpp \
   > /tmp/ethrxtb_build.log 2>&1 || { echo "BUILD FAILED"; grep -E '%Error' /tmp/ethrxtb_build.log | head; exit 1; }
./obj_dir_rxeng/Veth_rx_engine_loop_top 2>&1 | grep -E 'FAIL|PASS|drops='
