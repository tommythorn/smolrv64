#!/usr/bin/env bash
# systemd --version through ld.so under lockstep (see Makefile). The Ubuntu kernel reaches /init at
# ~1.05 G cycles; the loader's work on 16 MB of libraries follows.  CYC=0 for unbounded.
set -u
cd "$(dirname "$0")"
make -s || exit 1
INITRD=$(pwd)/tiny128-sysd.cpio OFF_INITRD=1e000000 DTB=$(pwd)/../tiny128/tiny128-cosim-sysd.dtb \
CYC=${CYC:-1600000000} ../../ooo2/run-ooo2-cosim-linux.sh
