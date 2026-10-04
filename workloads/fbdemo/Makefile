# fbdemo: a plasma on /dev/fb0 for the graphics console. No libc -- Ubuntu 26.04's riscv64 glibc
# is built for RVA23 (vector even in memset), which smolrv64 does not implement -- so this is
# plain rv64gc and runs on any riscv64 Linux: Ubuntu on the board, or simmerv --graphics.
CC = riscv64-linux-gnu-gcc
CFLAGS = -march=rv64gc -mabi=lp64d -O2 -Wall -Wextra -ffreestanding -fno-stack-protector \
         -fno-tree-loop-distribute-patterns -static -nostdlib -fno-pie -no-pie

fbdemo: fbdemo.c
	$(CC) $(CFLAGS) -o $@ $<

clean:
	rm -f fbdemo
