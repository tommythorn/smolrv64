#!/usr/bin/env bash
# The interrupt storm (docs/rtl-rules.md G7): the Linux lockstep cosim with -DOOO2_IRQ_STIM, a
# sim-only spurious level on the UART's PLIC source that rv_soc_top arms at the kernel's console
# handover and src/plic.v treats as enabled at priority 1. The 8250's fasteoi flow then runs ~45
# times per M cycles -- external-interrupt entry under a deferred CTF squash, PLIC claim and
# complete, sret -- and the harness follows the DUT's interrupts. No other simulation takes a
# PLIC interrupt at all (the tiny128 UART never enables RX; virtio is tied off), and the board
# reported the D12 defect as a dead NIC that this run finds in 447 M cycles. Run it after any
# change to redirect, squash, the LQ/LSU start gates, the irq FSM or the PLIC.
#
#   ooo2/run-ooo2-cosim-storm.sh                    # IW=2, 500 M cycles (the storm starts ~320 M)
#   IW=3 CYC=800000000 ooo2/run-ooo2-cosim-storm.sh
#   DDR_LAT=40 ooo2/run-ooo2-cosim-storm.sh         # a flat DDR latency: the interleavings differ
#
# The retire count has no expectation row at these cycle budgets (the handlers' work varies), so
# the verdict is the lockstep itself and the RTL's assertions: EXIT 0 means clean.
set -u
cd "$(dirname "$0")"
V="-DOOO2_IRQ_STIM"; [ -n "${IW:-}" ] && V="$V -DOOO2_IW=$IW"
PLUSARGS="${DDR_LAT:+ +ddr_lat=$DDR_LAT}${PLUSARGS:+ $PLUSARGS}" VDEFS="$V" CYC=${CYC:-500000000} ./run-ooo2-cosim-linux.sh
