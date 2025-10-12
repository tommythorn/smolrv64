        .globl _start
        .section .text.startup
_start: auipc sp,2
        j main
