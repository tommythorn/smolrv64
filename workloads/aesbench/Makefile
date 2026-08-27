# aesbench: deterministic T-table AES-128 (see aesbench.c). Boots at 0x8000_0000.
#   make && FW=$(pwd)/aesbench.bin ../../ooo2/run-ooo2-linux.sh
platform=$(shell uname -o)
ifeq ($(platform),Darwin)
  RV=riscv64-elf-
else
  RV=riscv64-linux-gnu-
endif

all: aesbench.bin

aesbench.bin: aesbench.elf
	$(RV)objcopy -O binary aesbench.elf aesbench.bin

aesbench.elf: aesbench.c crt0.s aesbench.ld
	$(RV)gcc -mabi=lp64d -mcmodel=medany -march=rv64gc_zicsr -fno-builtin -nostdlib \
		-Wl,-Taesbench.ld -O2 crt0.s aesbench.c -o aesbench.elf

clean:
	rm -f aesbench.elf aesbench.bin
