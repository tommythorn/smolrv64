# saxpybench: deterministic T-table AES-128 (see saxpybench.c). Boots at 0x8000_0000.
#   make && FW=$(pwd)/saxpybench.bin ../../ooo2/run-ooo2-linux.sh
platform=$(shell uname -o)
ifeq ($(platform),Darwin)
  RV=riscv64-elf-
else
  RV=riscv64-linux-gnu-
endif

all: saxpybench.bin

saxpybench.bin: saxpybench.elf
	$(RV)objcopy -O binary saxpybench.elf saxpybench.bin

saxpybench.elf: saxpybench.c crt0.s saxpybench.ld
	$(RV)gcc -mabi=lp64d -mcmodel=medany -march=rv64gc_zicsr -fno-builtin -nostdlib \
		-Wl,-Tsaxpybench.ld -O2 crt0.s saxpybench.c -o saxpybench.elf

clean:
	rm -f saxpybench.elf saxpybench.bin
