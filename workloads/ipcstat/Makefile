CC = riscv64-linux-gnu-gcc

# static + rv64gc: coffee's gcc defaults to an RVA23 baseline (emitted OP-V
# once and SIGILL'd on the board -- see ddrhpm); pin the arch.
all: ipcstat

ipcstat: ipcstat.c
	$(CC) -O2 -Wall -march=rv64gc -mabi=lp64d -static -o $@ $<

clean:
	rm -f ipcstat

.PHONY: all clean
