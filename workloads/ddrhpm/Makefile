CC = riscv64-linux-gnu-gcc

all: ddrhpm

# static: the board's rootfs may not have matching shared libs, and this must run
# in any environment (including a rescue shell) without dependencies.
# DYNAMIC, and rv64gc only: coffee's cross-libc carries vector instructions (its memcpy), so a
# static link SIGILLs on the board -- it did on 2026-09-04, again. Dynamic uses the board's libc.
ddrhpm: ddrhpm.c
	$(CC) -O2 -Wall -march=rv64gc -mabi=lp64d -o $@ $<
	@test $$(riscv64-linux-gnu-objdump -d $@ | grep -cE 'vset|vmv|vle|vse') = 0

clean:
	rm -f ddrhpm

.PHONY: all clean
