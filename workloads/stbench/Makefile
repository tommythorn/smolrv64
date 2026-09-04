# stbench: store throughput, L1-resident (see stbench.c). Boots at 0x8000_0000.
#   make && FW=$(pwd)/stbench.bin ../../ooo2/run-ooo2-linux.sh
platform=$(shell uname -o)
ifeq ($(platform),Darwin)
  RV=riscv64-elf-
else
  RV=riscv64-linux-gnu-
endif

all: stbench.bin

stbench.bin: stbench.elf
	$(RV)objcopy -O binary stbench.elf stbench.bin

stbench.elf: stbench.c crt0.s stbench.ld
	$(RV)gcc -mabi=lp64d -mcmodel=medany -march=rv64gc_zicsr -fno-builtin -nostdlib \
		-Wl,-Tstbench.ld -O2 crt0.s stbench.c -o stbench.elf

clean:
	rm -f stbench.elf stbench.bin
