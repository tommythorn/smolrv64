# fpbench: deterministic T-table AES-128 (see fpbench.c). Boots at 0x8000_0000.
#   make && FW=$(pwd)/fpbench.bin ../../ooo2/run-ooo2-linux.sh
platform=$(shell uname -o)
ifeq ($(platform),Darwin)
  RV=riscv64-elf-
else
  RV=riscv64-linux-gnu-
endif

all: fpbench.bin

fpbench.bin: fpbench.elf
	$(RV)objcopy -O binary fpbench.elf fpbench.bin

fpbench.elf: fpbench.c crt0.s fpbench.ld
	$(RV)gcc -mabi=lp64d -mcmodel=medany -march=rv64gc_zicsr -fno-builtin -nostdlib \
		-Wl,-Tfpbench.ld -O2 crt0.s fpbench.c -o fpbench.elf

clean:
	rm -f fpbench.elf fpbench.bin
