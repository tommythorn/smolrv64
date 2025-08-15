.globl start
start:
        lui     a1, 0x12345
        addi    a2, a1, 0x678
        auipc   a3, 0x10
        jal     dummy

        li      a0, 'H'
        csrw    0x666,a0
        li      a0, 'e'
        csrw    0x666,a0
        li      a0, 'l'
        csrw    0x666,a0
        csrw    0x666,a0
        li      a0, 'o'
        csrw    0x666,a0
        li      a0, '\r'
        csrw    0x666,a0
        li      a0, '\n'
        csrw    0x666,a0
        beq     x0,x0,start

dummy:  ret
