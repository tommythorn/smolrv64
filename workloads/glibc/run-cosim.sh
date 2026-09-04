#!/usr/bin/env bash
# Boot the glibc initramfs under lockstep. ~60 M cycles to the init hooks, then the test.
#   CYC=120000000 ./run-cosim.sh
set -u
cd "$(dirname "$0")"
make -s || exit 1
INITRD=$(pwd)/tiny128-glibc.cpio OFF_INITRD=1f000000 DTB=$(pwd)/../tiny128/tiny128-cosim-glibc.dtb \
CYC=${CYC:-120000000} ../../ooo2/run-ooo2-cosim-linux.sh
