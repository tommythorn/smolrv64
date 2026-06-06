        .option arch, +zicbop
        .section .text
        .globl _start
_start:
        li      a0, 0x80010000
        prefetch.r 0(a0)      # must execute as no-op (ORI x0 hint)
        prefetch.w 0(a0)
        prefetch.i 0(a0)
        li      t0, 1
        li      t1, 0x80001000
        sd      t0, 0(t1)     # tohost = 1 -> Test Passed (reached here = no trap)
0:      j 0b
