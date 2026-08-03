#!/bin/bash
# Build + run the interrupt end-to-end test: a tiny M-mode program (irqtest.S) that
# enables mie.MTIE+mstatus.MIE then spins; tb_irq.v holds MTIP (hw_ip[7]) high, so
# the core must inject+deliver a machine timer interrupt (mcause=(1<<63)|7) and run
# the handler. PASS = handler stores 1 to tohost. Proves the irq_take pseudo-op path.
set -e
cd "$(dirname "$0")"
GCC=${GCC:-riscv64-linux-gnu-gcc}; OC=${OC:-riscv64-linux-gnu-objcopy}; NM=${NM:-riscv64-linux-gnu-nm}
T=$(mktemp -d); trap 'rm -rf "$T"' EXIT
$GCC -nostdlib -fno-pic -mcmodel=medany -march=rv64imac_zicsr -mabi=lp64 -static -no-pie \
   -T irqtest.ld -o "$T/irq.elf" irqtest.S 2>/dev/null
$OC -O binary "$T/irq.elf" "$T/irq.bin"; od -An -v -tx1 "$T/irq.bin" > "$T/irq.hex"
TH=$($NM "$T/irq.elf" | awk '/ tohost$/{print $1}')
srcs=$(ls *.v | grep -vE '^tb_|probe|^flopwrap.v$|^rf_alu.v')
iverilog -g2012 -I. -s tb -o "$T/tb_irq.vvp" $srcs tb_irq.v ./alu.v ./smolrv64_sdpram.v fp_unit_stub.sv ./smolrv64_plic_arbiter.v
vvp "$T/tb_irq.vvp" +hex="$T/irq.hex" +tohost=$TH +cycles=6000
