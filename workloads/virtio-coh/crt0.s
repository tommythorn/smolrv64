        .globl _start
        .section .text.startup
        # Entered via the monitor's "X<addr>" — a plain call. Preserve the
        # monitor's sp/ra, run on our own stack, then return to the monitor
        # prompt so successive tests don't each need a board reset.
_start:
        mv      a4, sp                  # a4 = monitor sp (a4/a5 unused by monitor call)
        mv      a5, ra                  # a5 = monitor ra
        li      sp, 0x80080000          # our stack, below the rings at 0x80100000
        addi    sp, sp, -16
        sd      a4, 0(sp)
        sd      a5, 8(sp)
        call    main
        ld      a5, 8(sp)
        ld      sp, 0(sp)               # restore monitor sp (discards our frame)
        mv      ra, a5
        ret
