    .section .text
    .globl _start
_start:
    la   t0, handler
    csrw mtvec, t0
    li   t0, 0x80              # MTIE
    csrw mie, t0
    li   t0, 0x8               # MIE
    csrs mstatus, t0
spin:
    j    spin

    # keep the handler far from the spin loop so spin's fall-through
    # speculation lands on padding, not handler code.
    .balign 256
handler:
    csrr t0, mcause
    li   t1, 0x8000000000000007
    bne  t0, t1, fail
    li   t0, 1
    la   t1, tohost
    sd   t0, 0(t1)
hdone:
    j    hdone
fail:
    li   t0, 0xbad
    la   t1, tohost
    sd   t0, 0(t1)
fdone:
    j    fdone

    .section .tohost,"aw"
    .align 3
    .globl tohost
tohost: .dword 0
