#!/usr/bin/env bash
# The LONG-GUEST cosim: the Geekbench image (a full glibc userspace, 2 GiB) under lockstep.
#
#   ooo2/run-ooo2-cosim-gb5.sh            # unbounded: run until a divergence or Ctrl-C
#   CYC=500000000 ooo2/run-ooo2-cosim-gb5.sh
#
# tiny128 is 60 M cycles of a small guest and lines up little: on 2026-09-04 it was clean
# with and without a store-queue age defect that this image hit at retire 123,081,278
# (cycle ~447 M), after Ubuntu userspace had already segfaulted on the board. Its kernel
# boot alone is >400 M cycles (~25 min at ~4.6 ms of guest time per second), so this is a
# per-batch gate run in the background, not a per-commit one. Same script, same model
# stamp, same /tmp output as the tiny128 run -- do not run both at once.
set -u
cd "$(dirname "$0")"
G=../workloads/gb5
NAME=gb5 MEM_LG2=31 A1=9ff00000 OFF_DTB=1ff00000 OFF_INITRD=16270e00 \
FW=$G/fw_payload.bin DTB=$G/gb5.dtb INITRD=$G/img.Geekbench5.cpio CYC=${CYC:-0} \
exec ./run-ooo2-cosim-linux.sh
