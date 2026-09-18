# memrand: reset in M-mode, set up the stack and FP, clear .bss, hand over to main_m (C).
        .globl _start
        .section .text.startup
_start: li   sp, 0x80800000          # 8 MiB in: below the data windows at +16 MiB
        li   t0, 0x2000              # mstatus.FS = Initial before any compiled FP
        csrs mstatus, t0
        .option push
        .option norelax
        la   gp, __global_pointer$
        .option pop
        la   t0, __bss_start
        la   t1, _end
1:      bgeu t0, t1, 2f
        sd   zero, 0(t0)
        addi t0, t0, 8
        j    1b
2:      call main_m
3:      j    3b

# The S-mode trap entry: every integer register saved (the op stream keeps values in all of
# them), the C handler decides, then sret. FP registers are untouched by the handler.
        .globl s_trap_entry
        .align 4
s_trap_entry:
        addi sp, sp, -256
        sd x1, 8(sp);   sd x5, 40(sp);  sd x6, 48(sp);  sd x7, 56(sp);  sd x8, 64(sp)
        sd x9, 72(sp);  sd x10, 80(sp); sd x11, 88(sp); sd x12, 96(sp); sd x13, 104(sp)
        sd x14, 112(sp); sd x15, 120(sp); sd x16, 128(sp); sd x17, 136(sp); sd x18, 144(sp)
        sd x19, 152(sp); sd x20, 160(sp); sd x21, 168(sp); sd x22, 176(sp); sd x23, 184(sp)
        sd x24, 192(sp); sd x25, 200(sp); sd x26, 208(sp); sd x27, 216(sp); sd x28, 224(sp)
        sd x29, 232(sp); sd x30, 240(sp); sd x31, 248(sp)
        csrr a0, scause
        csrr a1, sepc
        csrr a2, stval
        call trap_c
        ld x1, 8(sp);   ld x5, 40(sp);  ld x6, 48(sp);  ld x7, 56(sp);  ld x8, 64(sp)
        ld x9, 72(sp);  ld x10, 80(sp); ld x11, 88(sp); ld x12, 96(sp); ld x13, 104(sp)
        ld x14, 112(sp); ld x15, 120(sp); ld x16, 128(sp); ld x17, 136(sp); ld x18, 144(sp)
        ld x19, 152(sp); ld x20, 160(sp); ld x21, 168(sp); ld x22, 176(sp); ld x23, 184(sp)
        ld x24, 192(sp); ld x25, 200(sp); ld x26, 208(sp); ld x27, 216(sp); ld x28, 224(sp)
        ld x29, 232(sp); ld x30, 240(sp); ld x31, 248(sp)
        addi sp, sp, 256
        sret
