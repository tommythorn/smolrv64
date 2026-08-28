# ldbench: deterministic T-table AES-128 (see ldbench.c). Boots at 0x8000_0000.
#   make && FW=$(pwd)/ldbench.bin ../../ooo2/run-ooo2-linux.sh
platform=$(shell uname -o)
ifeq ($(platform),Darwin)
  RV=riscv64-elf-
else
  RV=riscv64-linux-gnu-
endif

all: ldbench.bin

ldbench.bin: ldbench.elf
	$(RV)objcopy -O binary ldbench.elf ldbench.bin

ldbench.elf: ldbench.c crt0.s ldbench.ld
	$(RV)gcc -mabi=lp64d -mcmodel=medany -march=rv64gc_zicsr -fno-builtin -nostdlib \
		-Wl,-Tldbench.ld -O2 crt0.s ldbench.c -o ldbench.elf

clean:
	rm -f ldbench.elf ldbench.bin
