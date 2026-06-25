#!/bin/bash
# Build + run the SoC end-to-end test: soctest.S programs the real CLINT timer over
# MMIO (read mtime, write mtimecmp) then takes a machine timer interrupt delivered
# through tb_soc.v's address decoder (CLINT @ 0x0200_0000, mtip -> hw_ip[7]). PASS =
# handler stores 1 to tohost. Proves LSU<->CLINT MMIO routing end to end.
set -e
cd "$(dirname "$0")"
GCC=${GCC:-riscv64-linux-gnu-gcc}; OC=${OC:-riscv64-linux-gnu-objcopy}; NM=${NM:-riscv64-linux-gnu-nm}
T=$(mktemp -d); trap 'rm -rf "$T"' EXIT
$GCC -nostdlib -fno-pic -mcmodel=medany -march=rv64imac_zicsr -mabi=lp64 -static -no-pie \
   -T soctest.ld -o "$T/soc.elf" soctest.S 2>/dev/null
$OC -O binary "$T/soc.elf" "$T/soc.bin"; od -An -v -tx1 "$T/soc.bin" > "$T/soc.hex"
TH=$($NM "$T/soc.elf" | awk '/ tohost$/{print $1}')
srcs=$(ls *.v | grep -vE '^tb_|probe|^flopwrap.v$|^rf_alu.v')
iverilog -g2012 -I. -I../src -s tb -o "$T/tb_soc.vvp" $srcs tb_soc.v ../src/alu.v
vvp "$T/tb_soc.vvp" +hex="$T/soc.hex" +tohost=$TH +cycles=20000
