# febench: straight-line fetch bandwidth (see gen.py). Boots at 0x8000_0000.
#   make && FW=$(pwd)/febench.bin ../../ooo2/run-ooo2-linux.sh
platform=$(shell uname -o)
ifeq ($(platform),Darwin)
  RV=riscv64-elf-
else
  RV=riscv64-linux-gnu-
endif

all: febench.bin

febench_body.s: gen.py
	python3 gen.py > febench_body.s

febench.bin: febench.elf
	$(RV)objcopy -O binary febench.elf febench.bin

febench.elf: febench.c febench_body.s crt0.s febench.ld
	$(RV)gcc -mabi=lp64d -mcmodel=medany -march=rv64gc_zicsr -fno-builtin -nostdlib \
		-Wl,-Tfebench.ld -O2 crt0.s febench_body.s febench.c -o febench.elf

clean:
	rm -f febench.elf febench.bin
