        .globl _start
        .section .text.startup
_start:
        lla sp, __stack_top

        lla t0, __bss_start
        lla t1, _end
1:
        bgeu t0, t1, 2f
        sd zero, 0(t0)
        addi t0, t0, 8
        j 1b
2:
        j main
