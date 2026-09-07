#!/usr/bin/env bash
# Boot the glibc initramfs under lockstep. On the full two-wide stack (2026-09-07, the first
# run whose overlay actually unpacked -- see the Makefile) /init comes at ~0.83 G cycles, the
# test hook starts at ~1.1 G and each iteration is ~150 M cycles, so this is a per-batch run
# like the Geekbench one (~1.5 h), not a per-commit gate -- and the first customer of the
# cosim checkpoint (docs/PLAN-cosim-checkpoint.md). The per-iteration checksums are the
# yardstick: iteration=1 414556, iteration=2 131459 on main 12029862.
#   ./run-cosim.sh                 # 1.8 G cycles: init plus the 4 default iterations
#   CYC=0 ./run-cosim.sh           # unbounded
set -u
cd "$(dirname "$0")"
make -s || exit 1
INITRD=$(pwd)/tiny128-glibc.cpio OFF_INITRD=1f000000 DTB=$(pwd)/../tiny128/tiny128-cosim-glibc.dtb \
CYC=${CYC:-1800000000} ../../ooo2/run-ooo2-cosim-linux.sh
