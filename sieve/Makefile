sieve.bin: sieve.elf;riscv64-elf-objcopy -O binary sieve.elf sieve.bin
sieve.elf: sieve.c crt0.s;riscv64-elf-gcc -mcmodel=medany -march=rv64imac_zicsr -mabi=lp64 -fno-builtin -nostdlib -Wl,-Tsieve.ld -O crt0.s sieve.c -o sieve.elf

