# mlbench: deterministic T-table AES-128 (see mlbench.c). Boots at 0x8000_0000.
#   make && FW=$(pwd)/mlbench.bin ../../ooo2/run-ooo2-linux.sh
platform=$(shell uname -o)
ifeq ($(platform),Darwin)
  RV=riscv64-elf-
else
  RV=riscv64-linux-gnu-
endif

all: mlbench.bin

mlbench.bin: mlbench.elf
	$(RV)objcopy -O binary mlbench.elf mlbench.bin

mlbench.elf: mlbench.c crt0.s mlbench.ld
	$(RV)gcc -mabi=lp64d -mcmodel=medany -march=rv64gc_zicsr -fno-builtin -nostdlib \
		-Wl,-Tmlbench.ld -O2 crt0.s mlbench.c -o mlbench.elf

clean:
	rm -f mlbench.elf mlbench.bin
