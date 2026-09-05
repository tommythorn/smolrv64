# membench: store throughput, L1-resident (see membench.c). Boots at 0x8000_0000.
#   make && FW=$(pwd)/membench.bin ../../ooo2/run-ooo2-linux.sh
platform=$(shell uname -o)
ifeq ($(platform),Darwin)
  RV=riscv64-elf-
else
  RV=riscv64-linux-gnu-
endif

all: membench.bin

membench.bin: membench.elf
	$(RV)objcopy -O binary membench.elf membench.bin

membench.elf: membench.c crt0.s membench.ld
	$(RV)gcc -mabi=lp64d -mcmodel=medany -march=rv64gc_zicsr -fno-builtin -nostdlib \
		-Wl,-Tmembench.ld -O2 crt0.s membench.c -o membench.elf

clean:
	rm -f membench.elf membench.bin
