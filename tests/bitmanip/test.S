# Self-checking RVA22 (Zba/Zbb/Zbs) + Zicond test for smolrv64.
# Each new instruction is checked against a base-ISA equivalent (or a known
# constant), so no hand-computed 64-bit expected values. tohost protocol:
# store 1 = pass, (testnum<<1)|1 = fail at that check.
.section .text
.globl _start
_start:
        li   x1, 0x0123456789ABCDEF
        li   x2, 0xFEDCBA9876543210      # low 6 bits = 0x10 = 16 (shift/rot amount)
        li   x31, 1                       # test counter

        # ---- Zba ----
        sh1add x3, x1, x2
        slli x4, x1, 1
        add  x4, x4, x2
        bne  x3, x4, fail
        addi x31, x31, 1

        sh3add x3, x1, x2
        slli x4, x1, 3
        add  x4, x4, x2
        bne  x3, x4, fail
        addi x31, x31, 1

        add.uw x3, x1, x2                 # zext32(x1) + x2
        slli x4, x1, 32
        srli x4, x4, 32
        add  x4, x4, x2
        bne  x3, x4, fail
        addi x31, x31, 1

        slli.uw x3, x1, 4                 # zext32(x1) << 4
        slli x4, x1, 32
        srli x4, x4, 32
        slli x4, x4, 4
        bne  x3, x4, fail
        addi x31, x31, 1

        # ---- Zbb logic ----
        andn x3, x1, x2
        not  x4, x2
        and  x4, x1, x4
        bne  x3, x4, fail
        addi x31, x31, 1

        orn  x3, x1, x2
        not  x4, x2
        or   x4, x1, x4
        bne  x3, x4, fail
        addi x31, x31, 1

        xnor x3, x1, x2
        xor  x4, x1, x2
        not  x4, x4
        bne  x3, x4, fail
        addi x31, x31, 1

        # ---- Zbb min/max ----
        min  x3, x1, x2                   # x1>0, x2<0 (signed) -> min = x2
        bne  x3, x2, fail
        addi x31, x31, 1
        max  x3, x1, x2                   # -> x1
        bne  x3, x1, fail
        addi x31, x31, 1
        minu x3, x1, x2                   # x1<x2 unsigned -> x1
        bne  x3, x1, fail
        addi x31, x31, 1

        # ---- Zbb rotate ----
        ror  x3, x1, x2                   # rotate right by 16
        srli x4, x1, 16
        slli x5, x1, 48
        or   x4, x4, x5
        bne  x3, x4, fail
        addi x31, x31, 1

        rol  x3, x1, x2                   # rotate left by 16
        slli x4, x1, 16
        srli x5, x1, 48
        or   x4, x4, x5
        bne  x3, x4, fail
        addi x31, x31, 1

        # ---- Zbb count / extend / permute (constant checks) ----
        li   x8, 1
        clz  x3, x8
        li   x4, 63
        bne  x3, x4, fail
        addi x31, x31, 1

        li   x8, 8                        # ...1000
        ctz  x3, x8
        li   x4, 3
        bne  x3, x4, fail
        addi x31, x31, 1

        li   x8, 0xFF
        cpop x3, x8
        li   x4, 8
        bne  x3, x4, fail
        addi x31, x31, 1

        rev8 x3, x1
        li   x4, 0xEFCDAB8967452301
        bne  x3, x4, fail
        addi x31, x31, 1

        orc.b x3, x1                      # all bytes nonzero -> all ones
        li   x4, -1
        bne  x3, x4, fail
        addi x31, x31, 1

        sext.b x3, x1                     # low byte 0xEF -> -17
        li   x4, -17
        bne  x3, x4, fail
        addi x31, x31, 1

        zext.h x3, x1                     # low half 0xCDEF
        li   x4, 0xCDEF
        bne  x3, x4, fail
        addi x31, x31, 1

        # ---- Zbb word rotate (sign-extended) ----
        rorw x3, x1, x2                   # rotate x1[31:0] right by 16, sext
        li   x9, 0xFFFFFFFF
        and  x10, x1, x9                  # x1[31:0]
        srliw x4, x10, 16
        slliw x5, x10, 16
        or   x4, x4, x5
        sext.w x4, x4
        bne  x3, x4, fail
        addi x31, x31, 1

        # ---- Zbs ----
        bset x3, x1, x2                   # x1 | (1<<16)
        li   x6, 1
        andi x7, x2, 63
        sll  x6, x6, x7
        or   x4, x1, x6
        bne  x3, x4, fail
        addi x31, x31, 1

        bclr x3, x1, x2                   # x1 & ~(1<<16)
        li   x6, 1
        sll  x6, x6, x7
        not  x6, x6
        and  x4, x1, x6
        bne  x3, x4, fail
        addi x31, x31, 1

        binv x3, x1, x2                   # x1 ^ (1<<16)
        li   x6, 1
        sll  x6, x6, x7
        xor  x4, x1, x6
        bne  x3, x4, fail
        addi x31, x31, 1

        bext x3, x1, x2                   # (x1>>16)&1
        srl  x4, x1, x7
        andi x4, x4, 1
        bne  x3, x4, fail
        addi x31, x31, 1

        bexti x3, x1, 0                   # bit 0 of x1 = 1
        li   x4, 1
        bne  x3, x4, fail
        addi x31, x31, 1

        # ---- Zicond ----
        czero.eqz x3, x1, x2             # x2!=0 -> x1
        bne  x3, x1, fail
        addi x31, x31, 1
        czero.eqz x3, x1, x0             # cond==0 -> 0
        bne  x3, x0, fail
        addi x31, x31, 1
        czero.nez x3, x1, x2             # x2!=0 -> 0
        bne  x3, x0, fail
        addi x31, x31, 1
        czero.nez x3, x1, x0             # cond==0 -> x1
        bne  x3, x1, fail
        addi x31, x31, 1

pass:
        li   x4, 1
        la   x5, tohost
        sd   x4, 0(x5)
1:      j    1b

fail:
        slli x4, x31, 1
        ori  x4, x4, 1
        la   x5, tohost
        sd   x4, 0(x5)
1:      j    1b

.section .data
.align 3
.globl tohost
tohost: .dword 0
