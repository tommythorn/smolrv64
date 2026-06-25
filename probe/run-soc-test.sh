#!/bin/bash
# Build + run the SoC end-to-end tests against tb_soc.v (backend_top + behavioral RAM
# + real clint.v + NS16550A UART, behind an address decoder on the LSU memory port):
#   soctest  -- program the CLINT timer over MMIO, take a machine timer interrupt
#               (LSU<->CLINT load/store routing + mtip->hw_ip->trap)
#   uarttest -- print a string over the UART (sub-word byte MMIO loads+stores)
# PASS = the program stores 1 to tohost. Pass a test name to run just one.
set -e
cd "$(dirname "$0")"
GCC=${GCC:-riscv64-linux-gnu-gcc}; OC=${OC:-riscv64-linux-gnu-objcopy}; NM=${NM:-riscv64-linux-gnu-nm}
T=$(mktemp -d); trap 'rm -rf "$T"' EXIT

# build the harness once
srcs=$(ls *.v | grep -vE '^tb_|probe|^flopwrap.v$|^rf_alu.v')
iverilog -g2012 -I. -I../src -s tb -o "$T/tb_soc.vvp" $srcs tb_soc.v ../src/alu.v

run_one() {
   local name="$1"
   $GCC -nostdlib -fno-pic -mcmodel=medany -march=rv64imac_zicsr -mabi=lp64 -static -no-pie \
      -T soctest.ld -o "$T/$name.elf" "$name.S" 2>/dev/null
   $OC -O binary "$T/$name.elf" "$T/$name.bin"; od -An -v -tx1 "$T/$name.bin" > "$T/$name.hex"
   local th; th=$($NM "$T/$name.elf" | awk '/ tohost$/{print $1}')
   echo "=== $name ==="
   vvp "$T/tb_soc.vvp" +hex="$T/$name.hex" +tohost=$th +cycles=20000 2>&1 | grep -vE 'Not enough words'
}

tests="${*:-soctest uarttest}"
for t in $tests; do run_one "$t"; done
