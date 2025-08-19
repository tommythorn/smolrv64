.globl start
start:  csrr    t0,mcycle
        csrr    t0,minstret

        lui     s0, 0xF

        auipc   a3, 0
        lui     a1, 0x12345
        addi    a2, a1, 0x678
        jal     dummy

        csrrci  t0,mscratch,31
        csrrsi  t1,mscratch,2
        csrrw   t0,mscratch,t0
        csrrw   t0,mscratch,t0

        li      a0, 'H'
        li      a1, 'e'
        li      a2, 'l'
        li      a3, 'o'
        li      a4, '\r'
        li      a5, '\n'

        csrr    t0,mcycle
        csrr    t0,minstret

loop:   csrw    0x666,a0
        csrw    0x666,a1
        csrw    0x666,a2
        csrw    0x666,a2
        csrw    0x666,a3
        csrw    0x666,a4
        csrw    0x666,a5

        addi    s0, s0, -1
        bge     s0, x0, loop

        ebreak

dummy:  lb      x2,0(a3)
        lh      x3,6(a3)
        lw      x4,4(a3)

        la      x5, slot

        sb      x1, 1(x5)
        ld      x7, (x5)
        sh      x7, 6(x5)
        ld      x7, (x5)
        addi    x8,x7,0

        ret

slot:   .align 3
        .word   43
        .word   42
