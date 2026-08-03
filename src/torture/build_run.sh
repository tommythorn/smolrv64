#!/bin/bash
# Build the store->load torture and run it on the verilated probe (tb_vl).
#   CYC=<cycles> MEMLAT=<n> CACHE=<0|1> ./build_run.sh
set -eu
cd "$(dirname "$0")"
GCC=riscv64-linux-gnu-gcc; OBJCOPY=riscv64-linux-gnu-objcopy; NM=riscv64-linux-gnu-nm
$GCC -nostdlib -nostartfiles -march=rv64imafdc -mabi=lp64d -T link.ld -o torture.elf torture.S
$OBJCOPY -O binary torture.elf torture.bin
od -An -v -tx1 torture.bin > torture.hex
TH=$($NM torture.elf | awk '/ tohost$/{print $1}')
BIN=../obj_dir_vl/tb_vl
CYC=${CYC:-500000000}; MEMLAT=${MEMLAT:-1}; CACHE=${CACHE:-0}
echo "tohost=0x$TH  bin=$(wc -c < torture.bin)B  | cyc=$CYC memlat=$MEMLAT cache=$CACHE"
"$BIN" +hex=torture.hex +tohost="$TH" +cycles=$CYC +memlat=$MEMLAT +cache=$CACHE 2>&1 \
  | grep -aE 'RISCV-TEST|FATAL' | tail -3
