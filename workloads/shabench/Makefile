# shabench: the sha256sum instruction mix (OpenSSL's C fallback) with the board's 13 CPI-stack
# counters, bare metal for the testbench and a Linux build for perf on the board.
#   make                  both
#   make check            the Linux build's digest against python's hashlib
#   FW=$(pwd)/shabench.bin INITRD= CYC=30000000 ../../ooo2/run-ooo2-linux.sh
RV = riscv64-linux-gnu-

all: shabench.bin shabench shalib

shabench.bin: shabench.elf
	$(RV)objcopy -O binary shabench.elf shabench.bin

shabench.elf: shabench.c sha256.c sha256.h ../febench/crt0.s ../febench/febench.ld
	$(RV)gcc -mabi=lp64d -mcmodel=medany -march=rv64gc_zicsr -fno-builtin -nostdlib -DBARE \
		-Wl,-T../febench/febench.ld -O2 ../febench/crt0.s shabench.c sha256.c -o shabench.elf

# dynamic + rv64gc: coffee's gcc defaults to an RVA23 baseline that emits vector and SIGILLs
# on the board (see workloads/ipcstat); a dynamic link uses the board's own libc.
shabench: shabench.c sha256.c sha256.h
	$(RV)gcc -O2 -Wall -march=rv64gc -mabi=lp64d -o shabench shabench.c sha256.c

shalib: shalib.c
	$(RV)gcc -O2 -Wall -march=rv64gc -mabi=lp64d -o shalib shalib.c -ldl

check: shabench
	@python3 -c "import hashlib; b=bytes(((i*2654435761)>>24)&255 for i in range(65536)); print(hashlib.sha256(b*2).hexdigest(), ' reps=2'); print(hashlib.sha256(b*64).hexdigest(), ' reps=64')"

clean:
	rm -f shabench.elf shabench.bin shabench shalib
