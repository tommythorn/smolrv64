# blurbench: deterministic T-table AES-128 (see blurbench.c). Boots at 0x8000_0000.
#   make && FW=$(pwd)/blurbench.bin ../../ooo2/run-ooo2-linux.sh
platform=$(shell uname -o)
ifeq ($(platform),Darwin)
  RV=riscv64-elf-
else
  RV=riscv64-linux-gnu-
endif

all: blurbench.bin

blurbench.bin: blurbench.elf
	$(RV)objcopy -O binary blurbench.elf blurbench.bin

blurbench.elf: blurbench.c crt0.s blurbench.ld
	$(RV)gcc -mabi=lp64d -mcmodel=medany -march=rv64gc_zicsr -fno-builtin -nostdlib \
		-Wl,-Tblurbench.ld -O2 crt0.s blurbench.c -o blurbench.elf

clean:
	rm -f blurbench.elf blurbench.bin
