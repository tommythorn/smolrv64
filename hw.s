.globl start
start:  lui     s0, 0xF

        lui     a1, 0x12345
        addi    a2, a1, 0x678
        auipc   a3, 0x10
        jal     dummy

        li      a0, 'H'
        li      a1, 'e'
        li      a2, 'l'
        li      a3, 'o'
        li      a4, '\r'
        li      a5, '\n'

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

dummy:  ret
