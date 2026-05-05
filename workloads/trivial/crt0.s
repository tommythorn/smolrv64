        .globl _start
        .section .text.startup
_start: li sp, 0xFFFFFFF0
        j main
