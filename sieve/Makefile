RV=riscv64-elf-
RV=riscv64-linux-gnu-
sieve.bin: sieve.elf;$(RV)objcopy -O binary sieve.elf sieve.bin
sieve.elf: sieve.c crt0.s;$(RV)gcc -mcmodel=medany -march=rv64imac_zicsr -mabi=lp64 -fno-builtin -nostdlib -Wl,-Tsieve.ld -O crt0.s sieve.c -o sieve.elf

