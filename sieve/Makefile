platform=$(shell uname -o)

ifeq ($(platform),Darwin)
  RV=riscv64-elf-
else # Assume Linux
  RV=riscv64-linux-gnu-
endif

sieve.bin: sieve.elf
	$(RV)objcopy -O binary sieve.elf sieve.bin

sieve.elf: sieve.c crt0.s
	$(RV)gcc -mabi=lp64 -mcmodel=medany -march=rv64imac_zicsr -fno-builtin -nostdlib -Wl,-Tsieve.ld -O \
		crt0.s sieve.c -o sieve.elf
