CC = riscv64-linux-gnu-gcc

all: ddrhpm

# static: the board's rootfs may not have matching shared libs, and this must run
# in any environment (including a rescue shell) without dependencies.
ddrhpm: ddrhpm.c
	$(CC) -O2 -Wall -static -o $@ $<

clean:
	rm -f ddrhpm

.PHONY: all clean
