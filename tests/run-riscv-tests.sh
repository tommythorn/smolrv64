#!/bin/bash
# riscv-tests gate: delegates to the core's verilated suite (one binary, parallel classes,
# the real CVFPU). Success is `pass=240 fail=0`.
cd "$(dirname "$0")"
exec ../ooo2/run-ooo2-vl.sh "$@"
