platform=$(shell uname -o)

ifeq ($(platform),Darwin)
  RV=riscv64-elf-
else # Assume Linux
  RV=riscv64-linux-gnu-
endif

SRC_DIR = ../../src
MEM_BASE ?= 0x70000000
BUILD_STAMP := $(shell date +%Y%m%d%H%M%S)

monitor.bin: monitor.elf
	$(RV)objcopy -O binary monitor.elf monitor.bin

monitor.elf: monitor.c blake3.c blake3.h crt0.s monitor.ld
	$(RV)gcc -mabi=lp64 -mcmodel=medany -march=rv64imac_zicsr \
		-fno-builtin -fno-pic -fno-pie -fno-asynchronous-unwind-tables \
		-nostdlib -static -no-pie \
		-DMONITOR_BUILD_STAMP=0x$(BUILD_STAMP)ULL \
		-Wl,-Tmonitor.ld -Wl,--defsym,MEM_BASE=$(MEM_BASE) -O \
		crt0.s monitor.c blake3.c -o monitor.elf

monitor.even: monitor.bin
	$(SRC_DIR)/evenodd.py $< 0 > $@

monitor.odd: monitor.bin
	$(SRC_DIR)/evenodd.py $< 1 > $@




load: monitor.bin
	$(SRC_DIR)/evenodd.py monitor.bin 0 > $(SRC_DIR)/mem.even
	$(SRC_DIR)/evenodd.py monitor.bin 1 > $(SRC_DIR)/mem.odd

.PHONY: sim run load
