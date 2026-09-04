#!/usr/bin/env bash
# Boot the glibc initramfs under lockstep. At the measured DDR model the Ubuntu kernel reaches
# /init at ~1.05 G cycles (272 M retires, 5.5 s guest time), then each test iteration is ~50 M
# cycles, so this is a per-batch run like the Geekbench one (~1 h), not a per-commit gate --
# and the first customer of the cosim checkpoint (docs/PLAN-cosim-checkpoint.md).
#   ./run-cosim.sh                 # 1.4 G cycles: init plus the 4 default iterations
#   CYC=0 ./run-cosim.sh           # unbounded
set -u
cd "$(dirname "$0")"
make -s || exit 1
INITRD=$(pwd)/tiny128-glibc.cpio OFF_INITRD=1f000000 DTB=$(pwd)/../tiny128/tiny128-cosim-glibc.dtb \
CYC=${CYC:-1400000000} ../../ooo2/run-ooo2-cosim-linux.sh
