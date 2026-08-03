platform=$(shell uname -o)

ifeq ($(platform),Darwin)
  RV=riscv64-elf-
else # Assume Linux
  RV=riscv64-linux-gnu-
endif

SRC_DIR = ../../src

sieve.bin: sieve.elf
	$(RV)objcopy -O binary sieve.elf sieve.bin

sieve.elf: sieve.c crt0.s
	$(RV)gcc -mabi=lp64 -mcmodel=medany -march=rv64imac_zicsr -fno-builtin -nostdlib -Wl,-Tsieve.ld -O \
		crt0.s sieve.c -o sieve.elf

sieve.even: sieve.bin
	$(SRC_DIR)/evenodd.py $< 0 > $@

sieve.odd: sieve.bin
	$(SRC_DIR)/evenodd.py $< 1 > $@



load: sieve.bin
	$(SRC_DIR)/evenodd.py sieve.bin 0 > $(SRC_DIR)/mem.even
	$(SRC_DIR)/evenodd.py sieve.bin 1 > $(SRC_DIR)/mem.odd

.PHONY: sim load
