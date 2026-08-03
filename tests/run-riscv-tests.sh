#!/bin/bash
# riscv-tests gate, retargeted at the current core: delegates to the Verilator
# suite in src/ (one binary, parallel classes; the old sequential-core tester
# this script used to build was retired with that core).
cd "$(dirname "$0")"
exec ../src/run-vl-tests.sh "$@"
