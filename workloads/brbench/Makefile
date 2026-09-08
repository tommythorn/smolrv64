# brbench: store throughput, L1-resident (see brbench.c). Boots at 0x8000_0000.
#   make && FW=$(pwd)/brbench.bin ../../ooo2/run-ooo2-linux.sh
platform=$(shell uname -o)
ifeq ($(platform),Darwin)
  RV=riscv64-elf-
else
  RV=riscv64-linux-gnu-
endif

all: brbench.bin

brbench.bin: brbench.elf
	$(RV)objcopy -O binary brbench.elf brbench.bin

brbench.elf: brbench.c crt0.s brbench.ld
	$(RV)gcc -mabi=lp64d -mcmodel=medany -march=rv64gc_zicsr -fno-builtin -nostdlib \
		-Wl,-Tbrbench.ld -O2 crt0.s brbench.c -o brbench.elf

clean:
	rm -f brbench.elf brbench.bin
