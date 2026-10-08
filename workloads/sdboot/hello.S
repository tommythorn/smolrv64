# The SD autoboot simulation's payload: the monitor loads it from the disk image and jumps to
# it (X<addr>); it prints a line on the console and spins.
        .globl _start
        .section .text
_start:
        li      t0, 0x10000000          # the 16550
        lla     t1, msg
1:      lbu     t2, 0(t1)
        beqz    t2, 3f
2:      lbu     t3, 5(t0)               # LSR.THRE
        andi    t3, t3, 0x20
        beqz    t3, 2b
        sb      t2, 0(t0)
        addi    t1, t1, 1
        j       1b
3:      wfi
        j       3b
        .balign 8
msg:    .asciz  "SD-BOOT-OK\r\n"
